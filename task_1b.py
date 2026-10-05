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
# Tunables
# ----------------------------------------------------------------------------
BASE_SPEED = 12.0          # High cruise speed (rad/s)
MIN_APPROACH_SPEED = 1.8   # Approach crawl floor (rad/s)

# Stopping thresholds (calibrated for 0.22m corridor with front bumper offset)
FRONT_STOP = 0.090         # Stop and turn distance (m)
SLOW_DIST = 0.400          # Distance to begin progressive braking (m)

# Gyro heading lock (prevents drifting or spinning while moving forward)
K_HEAD = 4.0               # Proportional heading correction
K_GYRO = 0.6               # Damping on yaw rate
U_LIMIT = 2.0              # Max steering correction (rad/s)

# Turn tunables
TURN_RATE_MAX = 5.0        # Rapid rotation body rate (rad/s)
TURN_RATE_MIN = 1.0        # Creep turn speed for precise landing (rad/s)
TURN_KP = 6.0              # Turn P-gain
TURN_TOL = math.radians(2.0)
TURN_ANGLE = math.pi / 2.0 # 90 degrees

# ----------------------------------------------------------------------------
# State Machine
# ----------------------------------------------------------------------------
# Modes: 'DRIVE', 'STOP_AND_DECIDE', 'TURN'
mode = 'DRIVE'
accumulated_yaw = 0.0
target_angle = 0.0
stop_timer = 0


def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global mode, accumulated_yaw, target_angle, stop_timer

    data = json.loads(msg.payload.decode())

    fl = float(data["fl"])
    fr = float(data["fr"])
    sl = float(data["sl"])
    sr = float(data["sr"])
    yaw_rate = float(data["gyro"][2])
    dt = float(data["dt"]) if data.get("dt") and data["dt"] > 0 else 0.002

    front_dist = min(fl, fr)
    left_vel = 0.0
    right_vel = 0.0

    if mode == 'DRIVE':
        # Integrate heading to hold a straight line
        accumulated_yaw += yaw_rate * dt

        if front_dist <= FRONT_STOP:
            # Wall detected: cut speed immediately to settle
            mode = 'STOP_AND_DECIDE'
            stop_timer = 0
            left_vel = 0.0
            right_vel = 0.0
        else:
            # Progressive deceleration as robot approaches the wall
            if front_dist < SLOW_DIST:
                ratio = (front_dist - FRONT_STOP) / (SLOW_DIST - FRONT_STOP)
                ratio = max(0.0, min(1.0, ratio))
                speed = MIN_APPROACH_SPEED + (ratio ** 1.2) * (BASE_SPEED - MIN_APPROACH_SPEED)
            else:
                speed = BASE_SPEED

            # Active heading hold along the straight axis
            steering = (-K_HEAD * accumulated_yaw) - (K_GYRO * yaw_rate)
            steering = max(-U_LIMIT, min(U_LIMIT, steering))

            left_vel = speed - steering
            right_vel = speed + steering

    elif mode == 'STOP_AND_DECIDE':
        # Hold zero speed for ~20ms to kill linear momentum and settle sensor readings
        left_vel = 0.0
        right_vel = 0.0
        stop_timer += 1

        if stop_timer >= 10:
            accumulated_yaw = 0.0

            # Turn toward whichever side has greater open distance
            if sl >= sr:
                target_angle = TURN_ANGLE      # Turn Left (+90 deg)
            else:
                target_angle = -TURN_ANGLE     # Turn Right (-90 deg)

            mode = 'TURN'

    elif mode == 'TURN':
        accumulated_yaw += yaw_rate * dt
        err = target_angle - accumulated_yaw

        if abs(err) <= TURN_TOL:
            # Turn completed: reset heading tracker and resume high-speed drive
            accumulated_yaw = 0.0
            mode = 'DRIVE'
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED
        else:
            w = math.copysign(
                max(TURN_RATE_MIN, min(TURN_RATE_MAX, TURN_KP * abs(err))),
                err
            )
            # In-place rotation calculation
            wheel_speed = w * (TRACK / 2.0) / WHEEL_R
            left_vel = -wheel_speed
            right_vel = wheel_speed

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