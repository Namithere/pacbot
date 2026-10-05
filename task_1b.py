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
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

# --- PID & Navigation State ---
TARGET_WALL_DIST = 0.22      # Desired distance to the wall (m)
FRONT_THRESHOLD = 0.25       # Distance to detect obstacle in front (m)
OPENING_THRESHOLD = 0.45     # Side sensor threshold indicating wall has ended (m)

BASE_SPEED = 7.0             # Base forward speed (rad/s)
TURN_SPEED = 4.0             # In-place turn speed (rad/s)

KP = 22.0
KI = 0.2
KD = 1.2

integral_error = 0.0
prev_error = 0.0
current_yaw = 0.0
target_turn_angle = 0.0

# States: 'FORWARD', 'FOLLOW_WALL', 'TURN_CORNER', 'TURN_OPENING'
state = 'FORWARD'


def pid_controller(error, dt):
    global integral_error, prev_error
    integral_error += error * dt
    # Clamp integral to prevent windup
    integral_error = max(min(integral_error, 1.0), -1.0)
    derivative = (error - prev_error) / dt if dt > 0 else 0.0
    prev_error = error
    return (KP * error) + (KI * integral_error) + (KD * derivative)


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global state, current_yaw, target_turn_angle, integral_error, prev_error

    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings
    sr = data["sr"]            # Side-right ToF distance readings
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    # Integrate gyro rate to track heading relative to turn start
    current_yaw += yaw_rate * dt

    front_dist = min(fl, fr)
    left_vel = 0.0
    right_vel = 0.0

    if state == 'FORWARD':
        if front_dist <= FRONT_THRESHOLD:
            # Wall detected directly ahead: prepare in-place turn (default left: +pi/2)
            current_yaw = 0.0
            target_turn_angle = math.pi / 2.0
            integral_error = 0.0
            prev_error = 0.0
            state = 'TURN_CORNER'
            left_vel = -TURN_SPEED
            right_vel = TURN_SPEED
        else:
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED

    elif state == 'TURN_CORNER':
        # Turn until gyro registers ~90 degree rotation
        if current_yaw >= target_turn_angle:
            current_yaw = 0.0
            state = 'FOLLOW_WALL'
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED
        else:
            left_vel = -TURN_SPEED
            right_vel = TURN_SPEED

    elif state == 'FOLLOW_WALL':
        # Wall ends on the right: turn right into open corridor
        if sr > OPENING_THRESHOLD:
            current_yaw = 0.0
            target_turn_angle = -math.pi / 2.0
            state = 'TURN_OPENING'
            left_vel = TURN_SPEED
            right_vel = -TURN_SPEED

        # Wall detected ahead: turn left to avoid collision
        elif front_dist <= FRONT_THRESHOLD:
            current_yaw = 0.0
            target_turn_angle = math.pi / 2.0
            integral_error = 0.0
            prev_error = 0.0
            state = 'TURN_CORNER'
            left_vel = -TURN_SPEED
            right_vel = TURN_SPEED

        else:
            # PID tracking using right side distance
            err = sr - TARGET_WALL_DIST
            steering_adj = pid_controller(err, dt)

            left_vel = BASE_SPEED + steering_adj
            right_vel = BASE_SPEED - steering_adj

    elif state == 'TURN_OPENING':
        # Turning into the open branch (right turn)
        if current_yaw <= target_turn_angle:
            current_yaw = 0.0
            integral_error = 0.0
            prev_error = 0.0
            state = 'FORWARD'
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED
        else:
            left_vel = TURN_SPEED
            right_vel = -TURN_SPEED

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