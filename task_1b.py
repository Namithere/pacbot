import json
import math
import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883

TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"


# =========================================================
# PARAMETERS
# =========================================================

BASE_SPEED = 5.0
MAX_SPEED = 8.0
MIN_SPEED = 2.0

WALL_DISTANCE = 0.08

FRONT_STOP = 0.12
FRONT_SLOW = 0.35

WALL_DETECT = 0.30

TURN_SPEED = 3.5
TURN_ANGLE = math.pi / 2

TURN_TOLERANCE = math.radians(4)

MAX_RANGE = 2.0


# Wall PID
KP = 30.0
KI = 0.0
KD = 2.5

# Turn PID
TKP = 5.0
TKD = 0.25


# =========================================================
# PID
# =========================================================

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

        return max(
            -self.limit,
            min(self.limit, output)
        )


wall_pid = PID(
    KP,
    KI,
    KD,
    3.0
)

turn_pid = PID(
    TKP,
    0.0,
    TKD,
    TURN_SPEED
)


# =========================================================
# STATE
# =========================================================

mode = "DRIVE"

heading = 0.0
target_heading = 0.0

turn_direction = 1

wall_side = None


# =========================================================
# UTILITY
# =========================================================

def clean(x):

    try:
        x = float(x)
    except:
        return MAX_RANGE

    if math.isnan(x):
        return MAX_RANGE

    if math.isinf(x):
        return MAX_RANGE

    if x < 0:
        return MAX_RANGE

    return min(x, MAX_RANGE)


def clamp(x, low=-MAX_SPEED, high=MAX_SPEED):

    return max(low, min(high, x))


def angle_error(target, current):

    error = target - current

    while error > math.pi:
        error -= 2 * math.pi

    while error < -math.pi:
        error += 2 * math.pi

    return error


def get_speed(front):

    if front >= FRONT_SLOW:
        return BASE_SPEED

    if front <= FRONT_STOP:
        return MIN_SPEED

    ratio = (
        (front - FRONT_STOP) /
        (FRONT_SLOW - FRONT_STOP)
    )

    return MIN_SPEED + (
        BASE_SPEED - MIN_SPEED
    ) * ratio


# =========================================================
# START TURN
# =========================================================

def start_turn(sl, sr):

    global mode
    global target_heading
    global turn_direction

    # Turn towards the more open side
    if sl > sr:
        turn_direction = 1
    else:
        turn_direction = -1

    target_heading = heading + (
        turn_direction * TURN_ANGLE
    )

    while target_heading > math.pi:
        target_heading -= 2 * math.pi

    while target_heading < -math.pi:
        target_heading += 2 * math.pi

    turn_pid.reset()

    mode = "TURN"


# =========================================================
# CONTROLLER
# =========================================================

def controller(fl, fr, sl, sr, gyro_z, dt):

    global mode
    global heading
    global target_heading
    global wall_side

    fl = clean(fl)
    fr = clean(fr)
    sl = clean(sl)
    sr = clean(sr)

    dt = max(0.001, float(dt))

    # -----------------------------------------------------
    # Gyro integration
    # -----------------------------------------------------

    heading += gyro_z * dt

    while heading > math.pi:
        heading -= 2 * math.pi

    while heading < -math.pi:
        heading += 2 * math.pi

    front = min(fl, fr)

    # =====================================================
    # TURN MODE
    # =====================================================

    if mode == "TURN":

        error = angle_error(
            target_heading,
            heading
        )

        # Turn complete
        if abs(error) < TURN_TOLERANCE:

            mode = "DRIVE"

            wall_side = None

            wall_pid.reset()

            return 0.0, 0.0

        # PID turn
        turn = turn_pid.update(
            error,
            dt
        )

        # Guarantee enough turning power
        if abs(turn) < 1.5:
            turn = 1.5 if error > 0 else -1.5

        return (
            clamp(-turn, -TURN_SPEED, TURN_SPEED),
            clamp(turn, -TURN_SPEED, TURN_SPEED)
        )

    # =====================================================
    # FRONT WALL
    # =====================================================

    if front <= FRONT_STOP:

        start_turn(sl, sr)

        return 0.0, 0.0

    # =====================================================
    # SPEED
    # =====================================================

    speed = get_speed(front)

    # =====================================================
    # DETECT WALL
    # =====================================================

    left_wall = sl < WALL_DETECT
    right_wall = sr < WALL_DETECT

    # -----------------------------------------------------
    # BOTH WALLS
    # -----------------------------------------------------

    if left_wall and right_wall:

        if sl < sr:

            current_wall = "LEFT"
            distance = sl

        else:

            current_wall = "RIGHT"
            distance = sr

    # -----------------------------------------------------
    # LEFT WALL
    # -----------------------------------------------------

    elif left_wall:

        current_wall = "LEFT"
        distance = sl

    # -----------------------------------------------------
    # RIGHT WALL
    # -----------------------------------------------------

    elif right_wall:

        current_wall = "RIGHT"
        distance = sr

    # -----------------------------------------------------
    # NO WALL
    # -----------------------------------------------------

    else:

        current_wall = None
        distance = None

    # =====================================================
    # WALL FOLLOWING
    # =====================================================

    if current_wall is not None:

        if wall_side != current_wall:

            wall_pid.reset()

            wall_side = current_wall

        error = distance - WALL_DISTANCE

        correction = wall_pid.update(
            error,
            dt
        )

        # LEFT WALL
        if current_wall == "LEFT":

            left = speed - correction
            right = speed + correction

        # RIGHT WALL
        else:

            left = speed + correction
            right = speed - correction

        return (
            clamp(left),
            clamp(right)
        )

    # =====================================================
    # NO WALL
    # =====================================================

    wall_side = None

    # Continue straight
    return (
        clamp(speed),
        clamp(speed)
    )


# =========================================================
# MQTT
# =========================================================

def create_client():

    try:

        return mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2
        )

    except AttributeError:

        return mqtt.Client()


def on_connect(
    client,
    userdata,
    flags,
    reason_code,
    properties=None
):

    print("Connected to MQTT")

    client.subscribe(
        TOPIC_SENSORS
    )

    print(
        "Subscribed:",
        TOPIC_SENSORS
    )


def on_message(client, userdata, msg):

    try:

        data = json.loads(
            msg.payload.decode()
        )

        fl = data["fl"]
        fr = data["fr"]

        sl = data["sl"]
        sr = data["sr"]

        gyro_z = data["gyro"][2]

        dt = data["dt"]

        left, right = controller(
            fl,
            fr,
            sl,
            sr,
            gyro_z,
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

        print(
            "Controller error:",
            e
        )


# =========================================================
# MAIN
# =========================================================

def main():

    client = create_client()

    client.on_connect = on_connect
    client.on_message = on_message

    print("Starting PacBot controller...")

    client.connect(
        MQTT_HOST,
        MQTT_PORT,
        60
    )

    client.loop_forever()


if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print("Controller stopped")