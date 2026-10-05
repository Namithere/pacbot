"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back. Fill in your control logic where marked.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py
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
# Tunables (High-Speed & Active Counter-Braking)
# ----------------------------------------------------------------------------
BASE_SPEED = 14.0          # Aggressive forward speed (rad/s)
MIN_APPROACH_SPEED = 2.0   # Safe approach floor before stop (rad/s)
REVERSE_PULSE_SPEED = -5.0 # Active counter-torque to immediately cancel linear momentum

# Collision Avoidance Thresholds
FRONT_STOP = 0.108         # Trigger active brake to stop at ~0.06m after momentum (m)
SLOW_DIST = 0.550          # Extended braking window for high speed (m)
MAX_VALID = 1.0            # Discard non-finite / out-of-range sensor returns (m)

# Gyro heading stabilization during drive
K_HEAD = 7.0
K_GYRO = 1.2
U_LIMIT = 4.0

# Side wall bumper guard
SIDE_SAFE = 0.052
K_SIDE = 45.0

# Rotation parameters
TURN_RATE_MAX = 6.5        # Rapid in-place pivot (rad/s)
TURN_RATE_MIN = 1.5
TURN_KP = 8.0
TURN_TOL = math.radians(2.5)
TURN_ANGLE = math.pi / 2.0

# Timers
ACTIVE_BRAKE_TIME = 0.035  # Duration of reverse torque pulse (s)
SETTLE_TIME = 0.040        # Quick settle before sampling side sensors (s)

# ----------------------------------------------------------------------------
# State Machine
# ----------------------------------------------------------------------------
# States: 'DRIVE', 'ACTIVE_BRAKE', 'SAMPLE_SENSORS', 'TURN'
state = {
    "mode": "DRIVE",
    "t_mode": 0.0,
    "heading": 0.0,
    "angle": 0.0,
    "target": 0.0,
    "bias": 0.0,
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

    rate = yaw_rate - state["bias"]
    front = min(_val(fl), _val(fr))
    mode = state["mode"]
    left_vel = 0.0
    right_vel = 0.0

    if mode == "DRIVE":
        state["heading"] += rate * dt

        if front <= FRONT_STOP:
            # Wall reached: trigger active counter-brake to instantly kill forward inertia
            state["mode"] = "ACTIVE_BRAKE"
            state["t_mode"] = 0.0
            left_vel = REVERSE_PULSE_SPEED
            right_vel = REVERSE_PULSE_SPEED
        else:
            # Smooth progressive deceleration approaching the obstacle
            if front < SLOW_DIST:
                ratio = (front - FRONT_STOP) / (SLOW_DIST - FRONT_STOP)
                ratio = max(0.0, min(1.0, ratio))
                speed = MIN_APPROACH_SPEED + (ratio ** 1.3) * (BASE_SPEED - MIN_APPROACH_SPEED)
            else:
                speed = BASE_SPEED

            # Heading hold via gyro
            u = -K_HEAD * state["heading"] - K_GYRO * rate

            # Side clearance repulsion
            push = 0.0
            if _valid(sl) and sl < SIDE_SAFE:
                push -= K_SIDE * (SIDE_SAFE - sl)
            if _valid(sr) and sr < SIDE_SAFE:
                push += K_SIDE * (SIDE_SAFE - sr)

            if push != 0.0:
                state["heading"] = 0.0
            u = _clamp(u + push, -U_LIMIT, U_LIMIT)

            left_vel = speed - u
            right_vel = speed + u

    elif mode == "ACTIVE_BRAKE":
        state["t_mode"] += dt
        left_vel = REVERSE_PULSE_SPEED
        right_vel = REVERSE_PULSE_SPEED

        # After applying counter-torque pulse, switch to zero-velocity settle
        if state["t_mode"] >= ACTIVE_BRAKE_TIME:
            state["mode"] = "SAMPLE_SENSORS"
            state["t_mode"] = 0.0
            left_vel = 0.0
            right_vel = 0.0

    elif mode == "SAMPLE_SENSORS":
        state["t_mode"] += dt
        left_vel = 0.0
        right_vel = 0.0

        if state["t_mode"] >= SETTLE_TIME:
            # Measure side distances while stopped and turn toward the open passage
            left_space = _val(sl)
            right_space = _val(sr)

            state["target"] = TURN_ANGLE if left_space >= right_space else -TURN_ANGLE
            state["angle"] = 0.0
            state["mode"] = "TURN"

    elif mode == "TURN":
        state["angle"] += rate * dt
        err = state["target"] - state["angle"]

        if abs(err) < TURN_TOL:
            # Turn complete: instantly launch back into forward drive
            state["mode"] = "DRIVE"
            state["heading"] = 0.0
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED
        else:
            w = math.copysign(
                _clamp(TURN_KP * abs(err), TURN_RATE_MIN, TURN_RATE_MAX),
                err
            )
            wheel = w * (TRACK / 2.0) / WHEEL_R
            left_vel, right_vel = -wheel, wheel

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