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

# --- Navigation Parameters ---
MAX_SPEED = 4.5              # Controlled forward cruise speed (rad/s)
MIN_SPEED = 0.8              # Creep speed when nearing the wall (rad/s)
TURN_SPEED = 2.5             # Safe in-place rotation speed (rad/s)

# Safe stopping distance inside a 0.22m corridor
STOP_DIST = 0.085            # Triggers brake and turn (m)
SLOW_DIST = 0.350            # Distance where smooth braking starts (m)

# --- State Machine Tracking ---
# States: 'MOVE_FORWARD', 'BRAKE_STOP', 'TURNING'
state = 'MOVE_FORWARD'
accumulated_yaw = 0.0
target_angle = 0.0
stop_counter = 0


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global state, accumulated_yaw, target_angle, stop_counter

    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings
    sr = data["sr"]            # Side-right ToF distance readings
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    front_dist = min(fl, fr)
    left_vel = 0.0
    right_vel = 0.0

    if state == 'MOVE_FORWARD':
        if front_dist <= STOP_DIST:
            # Active brake to eliminate linear inertia before turning
            state = 'BRAKE_STOP'
            stop_counter = 0
            left_vel = 0.0
            right_vel = 0.0
        else:
            # Linear deceleration as the wall gets closer
            if front_dist < SLOW_DIST:
                ratio = (front_dist - STOP_DIST) / (SLOW_DIST - STOP_DIST)
                ratio = max(0.0, min(1.0, ratio))
                speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * ratio
            else:
                speed = MAX_SPEED

            left_vel = speed
            right_vel = speed

    elif state == 'BRAKE_STOP':
        # Hold zero velocity for 5 simulation ticks (~10ms) to settle physics
        left_vel = 0.0
        right_vel = 0.0
        stop_counter += 1

        if stop_counter >= 5:
            accumulated_yaw = 0.0
            # Read side clearance while stationary to choose the open path
            if sl > sr:
                target_angle = math.pi / 2.0   # Turn Left
                left_vel = -TURN_SPEED
                right_vel = TURN_SPEED
            else:
                target_angle = -math.pi / 2.0  # Turn Right
                left_vel = TURN_SPEED
                right_vel = -TURN_SPEED

            state = 'TURNING'

    elif state == 'TURNING':
        accumulated_yaw += yaw_rate * dt

        is_turn_done = False
        if target_angle > 0:  # Turning Left
            if accumulated_yaw >= target_angle:
                is_turn_done = True
            else:
                left_vel = -TURN_SPEED
                right_vel = TURN_SPEED
        else:  # Turning Right
            if accumulated_yaw <= target_angle:
                is_turn_done = True
            else:
                left_vel = TURN_SPEED
                right_vel = -TURN_SPEED

        if is_turn_done:
            accumulated_yaw = 0.0
            state = 'MOVE_FORWARD'
            left_vel = MIN_SPEED
            right_vel = MIN_SPEED

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