import json
import math
import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883

TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ================= CONTROLLER SETTINGS =================

BASE_SPEED = 6.0
MAX_WHEEL = 10.0
MIN_SPEED = 2.0

MAX_RANGE = 2.0

WALL_TARGET = 0.08
FRONT_STOP = 0.10
BRAKE_ZONE = 0.40

OPEN_THRESH = 0.25

TURN_ANGLE = math.pi / 2
TURN_TOL = math.radians(3)

STOP_TIME = 0.10

# Wall PID
WALL_KP = 35.0
WALL_KI = 0.0
WALL_KD = 3.0

# Turn PID
TURN_KP = 5.0
TURN_KI = 0.0
TURN_KD = 0.25

# Heading PID
HOLD_KP = 3.0
HOLD_KI = 0.0
HOLD_KD = 0.15


# ================= PID =================

class PID:

    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        self.integral = 0.0
        self.previous = None

    def reset(self):
        self.integral = 0.0
        self.previous = None

    def update(self, error, dt):

        if dt <= 0:
            return 0.0

        self.integral += error * dt

        if self.previous is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous) / dt

        self.previous = error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        return max(-self.limit, min(self.limit, output))


wall_pid = PID(
    WALL_KP,
    WALL_KI,
    WALL_KD,
    BASE_SPEED
)

turn_pid = PID(
    TURN_KP,
    TURN_KI,
    TURN_KD,
    BASE_SPEED
)

hold_pid = PID(
    HOLD_KP,
    HOLD_KI,
    HOLD_KD,
    BASE_SPEED / 2
)


# ================= STATE =================

state = {

    "mode": "DRIVE",

    "heading": 0.0,

    "target_heading": 0.0,

    "turn_direction": 0.0,

    "stop_timer": 0.0,

    "wall_side": None
}


# ================= UTILITY =================

def clean(value):

    try:
        value = float(value)
    except:
        return MAX_RANGE

    if math.isnan(value):
        return MAX_RANGE

    if math.isinf(value):
        return MAX_RANGE

    if value < 0:
        return MAX_RANGE

    return min(value, MAX_RANGE)


def clamp(value):

    return max(-MAX_WHEEL, min(MAX_WHEEL, value))


def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2 * math.pi

    while angle < -math.pi:
        angle += 2 * math.pi

    return angle


def speed_from_front(front):

    if front >= FRONT_STOP + BRAKE_ZONE:
        return BASE_SPEED

    ratio = (
        (front - FRONT_STOP) /
        BRAKE_ZONE
    )

    ratio = max(0.0, min(1.0, ratio))

    return MIN_SPEED + (
        BASE_SPEED - MIN_SPEED
    ) * ratio


# ================= START TURN =================

def start_turn(sl, sr):

    if sl >= sr:
        state["turn_direction"] = 1.0
    else:
        state["turn_direction"] = -1.0

    state["stop_timer"] = STOP_TIME
    state["mode"] = "STOP"

    wall_pid.reset()
    hold_pid.reset()


# ================= CONTROLLER =================

def controller(fl, fr, sl, sr, yaw_rate, dt):

    fl = clean(fl)
    fr = clean(fr)
    sl = clean(sl)
    sr = clean(sr)

    dt = max(0.001, float(dt))

    # Integrate gyro
    state["heading"] += yaw_rate * dt

    state["heading"] = normalize_angle(
        state["heading"]
    )

    front = min(fl, fr)

    mode = state["mode"]

    # ==================================================
    # STOP
    # ==================================================

    if mode == "STOP":

        state["stop_timer"] -= dt

        if state["stop_timer"] <= 0:

            state["target_heading"] = normalize_angle(
                state["heading"] +
                state["turn_direction"] * TURN_ANGLE
            )

            turn_pid.reset()

            state["mode"] = "TURN"

        return 0.0, 0.0

    # ==================================================
    # TURN
    # ==================================================

    if mode == "TURN":

        error = normalize_angle(
            state["target_heading"] -
            state["heading"]
        )

        if (
            abs(error) < TURN_TOL and
            abs(yaw_rate) < 0.25
        ):

            state["heading"] = state["target_heading"]

            state["mode"] = "DRIVE"

            state["wall_side"] = None

            wall_pid.reset()
            hold_pid.reset()

            return 0.0, 0.0

        turn = turn_pid.update(
            error,
            dt
        )

        turn *= state["turn_direction"]

        return (
            clamp(-turn),
            clamp(turn)
        )

    # ==================================================
    # FRONT WALL
    # ==================================================

    if front <= FRONT_STOP:

        start_turn(sl, sr)

        return 0.0, 0.0

    # ==================================================
    # SPEED
    # ==================================================

    speed = speed_from_front(front)

    # ==================================================
    # WALL DETECTION
    # ==================================================

    left_wall = sl < OPEN_THRESH
    right_wall = sr < OPEN_THRESH

    # Select wall
    if left_wall and right_wall:

        if sl < sr:
            wall = sl
            wall_side = "LEFT"
        else:
            wall = sr
            wall_side = "RIGHT"

    elif left_wall:

        wall = sl
        wall_side = "LEFT"

    elif right_wall:

        wall = sr
        wall_side = "RIGHT"

    else:

        wall = None
        wall_side = None

    # ==================================================
    # WALL FOLLOWING
    # ==================================================

    if wall is not None:

        if state["wall_side"] != wall_side:

            wall_pid.reset()

            state["wall_side"] = wall_side

        error = wall - WALL_TARGET

        correction = wall_pid.update(
            error,
            dt
        )

        # LEFT WALL
        if wall_side == "LEFT":

            left_speed = speed - correction
            right_speed = speed + correction

        # RIGHT WALL
        else:

            left_speed = speed + correction
            right_speed = speed - correction

        return (
            clamp(left_speed),
            clamp(right_speed)
        )

    # ==================================================
    # NO WALL → HOLD HEADING
    # ==================================================

    state["wall_side"] = None

    error = normalize_angle(
        state["target_heading"] -
        state["heading"]
    )

    correction = hold_pid.update(
        error,
        dt
    )

    return (
        clamp(speed - correction),
        clamp(speed + correction)
    )


# ================= MQTT =================

def create_client():

    try:

        return mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2
        )

    except AttributeError:

        return mqtt.Client()


def on_message(client, userdata, msg):

    try:

        data = json.loads(
            msg.payload.decode()
        )

        fl = data["fl"]
        fr = data["fr"]

        sl = data["sl"]
        sr = data["sr"]

        gyro = data["gyro"]

        yaw_rate = gyro[2]

        dt = data["dt"]

        left, right = controller(
            fl,
            fr,
            sl,
            sr,
            yaw_rate,
            dt
        )

        command = {

            "left": float(left),

            "right": float(right)
        }

        client.publish(
            TOPIC_WHEEL_VEL,
            json.dumps(command)
        )

    except Exception as e:

        print("Controller error:", e)


# ================= MAIN =================

def main():

    client = create_client()

    client.on_message = on_message

    client.connect(
        MQTT_HOST,
        MQTT_PORT
    )

    client.subscribe(
        TOPIC_SENSORS
    )

    print("PacBot controller started")

    client.loop_forever()


if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("\nController stopped")