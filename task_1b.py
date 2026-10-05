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

# --- Navigation & Speed Parameters ---
BASE_SPEED = 8.0             # Set to base speed 8
CRAWL_SPEED = 1.0            # Speed for the final approach
TURN_SPEED = 3.5             # In-place rotation speed

# Corridor & Stopping Thresholds
STOP_DIST = 0.050            # Stops at 0.05m from the wall
DECEL_DIST = 0.220           # Begin braking within cell distance

# --- State Machine Tracking ---
# States: 'MOVE_FORWARD', 'STOPPED', 'TURNING'
state = 'MOVE_FORWARD'
accumulated_yaw = 0.0
target_angle = 0.0
stop_delay_ticks = 0


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global state, accumulated_yaw, target_angle, stop_delay_ticks

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
            # 1. Stop completely before taking the turn
            state = 'STOPPED'
            stop_delay_ticks = 0
            left_vel = 0.0
            right_vel = 0.0
        else:
            # Smooth proportional deceleration to stop cleanly at 0.05m
            if front_dist < DECEL_DIST:
                ratio = (front_dist - STOP_DIST) / (DECEL_DIST - STOP_DIST)
                ratio = max(0.0, min(1.0, ratio))
                speed = CRAWL_SPEED + (BASE_SPEED - CRAWL_SPEED) * ratio
            else:
                speed = BASE_SPEED

            left_vel = speed
            right_vel = speed

    elif state == 'STOPPED':
        # Ensure wheels are completely stationary and eliminate forward momentum
        left_vel = 0.0
        right_vel = 0.0
        stop_delay_ticks += 1

        # Hold stationary for a brief settle window (~16ms at 500Hz)
        if stop_delay_ticks >= 8:
            accumulated_yaw = 0.0

            # 2. Check which wall is farther away (left vs right) and target 90 degrees
            if sl > sr:
                target_angle = math.pi / 2.0   # Left turn (+90 deg)
                left_vel = -TURN_SPEED
                right_vel = TURN_SPEED
            else:
                target_angle = -math.pi / 2.0  # Right turn (-90 deg)
                left_vel = TURN_SPEED
                right_vel = -TURN_SPEED

            state = 'TURNING'

    elif state == 'TURNING':
        # Integrate gyro angular velocity over time
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

        # 3. Once 90 degrees completed, continue straight at base speed
        if is_turn_done:
            accumulated_yaw = 0.0
            state = 'MOVE_FORWARD'
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED

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