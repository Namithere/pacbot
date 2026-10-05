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

# --- Navigation Parameters ---
MAX_SPEED = 6.0              # Regular cruising speed (rad/s)
MIN_SPEED = 1.2              # Creep speed as approaching the wall (rad/s)
TURN_SPEED = 3.5             # In-place turn speed (rad/s)

# Corridor is 0.22m wide; stop halfway before hitting the perpendicular wall
STOP_DIST = 0.115            # Distance threshold to stop and turn (m)
SLOW_DIST = 0.28             # Distance threshold to start smooth deceleration (m)

# --- State Machine Tracking ---
# States: 'MOVE_FORWARD', 'TURNING'
state = 'MOVE_FORWARD'
accumulated_yaw = 0.0
target_angle = 0.0


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global state, accumulated_yaw, target_angle

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
        # Wall is directly ahead: stop and turn towards the open direction
        if front_dist <= STOP_DIST:
            accumulated_yaw = 0.0

            # Turn toward whichever side has the wall farthest away
            if sl > sr:
                # Turn Left (+pi/2)
                target_angle = math.pi / 2.0
                left_vel = -TURN_SPEED
                right_vel = TURN_SPEED
            else:
                # Turn Right (-pi/2)
                target_angle = -math.pi / 2.0
                left_vel = TURN_SPEED
                right_vel = -TURN_SPEED

            state = 'TURNING'
        else:
            # Smooth proportional deceleration to prevent crashing
            if front_dist < SLOW_DIST:
                ratio = (front_dist - STOP_DIST) / (SLOW_DIST - STOP_DIST)
                ratio = max(0.0, min(1.0, ratio))
                speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * ratio
            else:
                speed = MAX_SPEED

            left_vel = speed
            right_vel = speed

    elif state == 'TURNING':
        # Integrate gyro angular velocity over time
        accumulated_yaw += yaw_rate * dt

        # Check whether target 90-degree rotation is complete
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