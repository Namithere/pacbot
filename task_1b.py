"""PB Task 1B - Straight driving, stop at front wall, 90-degree turn toward
the side with more clearance, and repeat until exit.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b.py
"""
import json
import math
import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ----------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------
WHEEL_R = 0.017
TRACK = 0.092

# ----------------------------------------------------------------------------
# Tunables
# ----------------------------------------------------------------------------
BASE_SPEED = 8.0          # Base forward cruise speed (rad/s)
MIN_SPEED = 1.2           # Minimum approach crawl speed (rad/s)

# Front wall detection calibrated for 0.22m corridor
FRONT_STOP = 0.090        # Stop and begin turn sequence below this (m)
FRONT_EMERGENCY = 0.065   # Instant brake threshold (m)
SLOW_DIST = 0.280         # Begin proportional deceleration (m)
FRONT_CONFIRM = 2         # Consecutive readings below FRONT_STOP to trigger brake
MAX_VALID = 1.0           # Maximum valid sensor distance (m)

# Gyro heading hold during straight drive
K_HEAD = 5.0              # Heading proportional correction
K_GYRO = 0.8              # Heading rate damping
U_LIMIT = 2.5             # Maximum differential steering adjustment (rad/s)

# Wall proximity protection while driving forward
SIDE_SAFE = 0.050         # Distance buffer from corridor side walls (m)
K_SIDE = 35.0             # Side repulsion gain

# Stop-and-turn parameters
BRAKE_TIME = 0.15         # Settle time at 0 speed before reading side sensors (s)
BIAS_TIME = 0.10          # Gyro bias sampling window while stationary (s)
TURN_RATE_MAX = 2.8       # Maximum angular turn speed (rad/s)
TURN_RATE_MIN = 0.8       # Minimum angular turn speed (rad/s)
TURN_KP = 4.0             # Proportional gain for rotation
TURN_TOL = math.radians(1.5)
SETTLE_TIME = 0.12        # Settle duration after completing turn (s)
SETTLE_TOL = math.radians(2.5)
TURN_ANGLE = math.pi / 2  # 90 degrees (+left, -right)

# ----------------------------------------------------------------------------
# State Machine
# ----------------------------------------------------------------------------
state = {
    "mode": "DRIVE",      # DRIVE -> BRAKE -> TURN -> SETTLE -> DRIVE
    "t": 0.0,
    "t_mode": 0.0,
    "front_count": 0,
    "heading": 0.0,
    "angle": 0.0,
    "target": 0.0,
    "bias": 0.0,
    "bias_sum": 0.0,
    "sl_sum": 0.0,
    "sr_sum": 0.0,
    "n": 0,
}


def _valid(x):
    return x is not None and math.isfinite(x) and 0.0 < x < MAX_VALID


def _val(x):
    return x if _valid(x) else MAX_VALID


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def _start_brake():
    state.update(
        mode="BRAKE",
        t_mode=0.0,
        angle=0.0,
        front_count=0,
        bias_sum=0.0,
        sl_sum=0.0,
        sr_sum=0.0,
        n=0
    )


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    fl = data["fl"]
    fr = data["fr"]
    sl = data["sl"]
    sr = data["sr"]
    yaw_rate = data["gyro"][2]
    dt = data["dt"]

    if dt is None or dt <= 0:
        dt = 0.002

    state["t"] += dt
    rate = yaw_rate - state["bias"]

    # Minimum of valid readings ensures early detection even when approaching at an angle
    front = min(_val(fl), _val(fr))
    mode = state["mode"]
    left_vel = 0.0
    right_vel = 0.0

    if mode == "DRIVE":
        state["heading"] += rate * dt

        # Wall detection confirmation
        if front < FRONT_STOP:
            state["front_count"] += 1
        else:
            state["front_count"] = 0

        # Trigger braking on confirmation or emergency proximity
        if state["front_count"] >= FRONT_CONFIRM or front <= FRONT_EMERGENCY:
            _start_brake()
            left_vel = 0.0
            right_vel = 0.0
        else:
            # Proportional deceleration approaching the wall
            if front < SLOW_DIST:
                k = _clamp((front - FRONT_STOP) / (SLOW_DIST - FRONT_STOP), 0.0, 1.0)
                speed = MIN_SPEED + k * (BASE_SPEED - MIN_SPEED)
            else:
                speed = BASE_SPEED

            # Heading hold using gyro
            u = -K_HEAD * state["heading"] - K_GYRO * rate

            # Side clearance push away from side walls
            push = 0.0
            if SIDE_SAFE > 0:
                if _valid(sl) and sl < SIDE_SAFE:
                    push -= K_SIDE * (SIDE_SAFE - sl)
                if _valid(sr) and sr < SIDE_SAFE:
                    push += K_SIDE * (SIDE_SAFE - sr)

            if push != 0.0:
                state["heading"] = 0.0
            u = _clamp(u + push, -U_LIMIT, U_LIMIT)

            left_vel = speed - u
            right_vel = speed + u

    elif mode == "BRAKE":
        left_vel = 0.0
        right_vel = 0.0
        state["t_mode"] += dt

        # Settle robot and integrate accurate stationary readings
        if state["t_mode"] >= BRAKE_TIME:
            state["bias_sum"] += yaw_rate
            state["sl_sum"] += _val(sl)
            state["sr_sum"] += _val(sr)
            state["n"] += 1

            if state["t_mode"] >= BRAKE_TIME + BIAS_TIME:
                n = max(state["n"], 1)
                state["bias"] = state["bias_sum"] / n
                left_space = state["sl_sum"] / n
                right_space = state["sr_sum"] / n

                # Turn toward the side with greater open distance
                state["target"] = TURN_ANGLE if left_space >= right_space else -TURN_ANGLE
                state["angle"] = 0.0
                state["mode"] = "TURN"

    elif mode == "TURN":
        state["angle"] += rate * dt
        err = state["target"] - state["angle"]

        if abs(err) < TURN_TOL:
            state.update(mode="SETTLE", t_mode=0.0)
            left_vel = 0.0
            right_vel = 0.0
        else:
            w = math.copysign(
                _clamp(TURN_KP * abs(err), TURN_RATE_MIN, TURN_RATE_MAX),
                err
            )
            # In-place differential wheel rotation
            wheel = w * (TRACK / 2.0) / WHEEL_R
            left_vel, right_vel = -wheel, wheel

    elif mode == "SETTLE":
        state["angle"] += rate * dt
        state["t_mode"] += dt
        left_vel = 0.0
        right_vel = 0.0

        if state["t_mode"] >= SETTLE_TIME:
            if abs(state["target"] - state["angle"]) > SETTLE_TOL:
                state["mode"] = "TURN"
            else:
                state.update(mode="DRIVE", heading=0.0, front_count=0)

    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass