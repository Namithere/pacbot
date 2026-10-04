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


class ControllerState:
    def __init__(self):
        self.state = "FOLLOW"
        self.turn_direction = None
        self.accumulated_yaw = 0.0
        self.target_turn_angle = 0.0

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # Thresholds (meters)
        self.target_dist = 0.12
        self.front_wall_dist = 0.18
        self.max_side_range = 0.30

        # Wheel speeds (rad/s)
        self.base_speed = 25.0
        self.turn_speed = 14.0

        # PID gains
        self.Kp = 45.0
        self.Ki = 0.1
        self.Kd = 2.0


ctrl = ControllerState()


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings
    sr = data["sr"]            # Side-right ToF distance readings
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    # ========================== CONTROLLER LOGIC ==========================
    left_vel = 0.0
    right_vel = 0.0

    front_dist = min(fl, fr)

    if ctrl.state == "TURN":
        ctrl.accumulated_yaw += yaw_rate * dt

        if abs(ctrl.accumulated_yaw) >= ctrl.target_turn_angle:
            ctrl.state = "FOLLOW"
            ctrl.accumulated_yaw = 0.0
            ctrl.integral = 0.0
            ctrl.prev_error = 0.0
        else:
            if ctrl.turn_direction == "LEFT":
                left_vel = -ctrl.turn_speed
                right_vel = ctrl.turn_speed
            else:
                left_vel = ctrl.turn_speed
                right_vel = -ctrl.turn_speed

    elif ctrl.state == "FOLLOW":
        if front_dist < ctrl.front_wall_dist:
            ctrl.state = "TURN"
            ctrl.accumulated_yaw = 0.0
            ctrl.integral = 0.0
            ctrl.prev_error = 0.0

            if sl > sr and sl > ctrl.target_dist:
                ctrl.turn_direction = "LEFT"
                ctrl.target_turn_angle = math.pi / 2.0
            elif sr > ctrl.target_dist:
                ctrl.turn_direction = "RIGHT"
                ctrl.target_turn_angle = math.pi / 2.0
            else:
                ctrl.turn_direction = "LEFT"
                ctrl.target_turn_angle = math.pi

            if ctrl.turn_direction == "LEFT":
                left_vel = -ctrl.turn_speed
                right_vel = ctrl.turn_speed
            else:
                left_vel = ctrl.turn_speed
                right_vel = -ctrl.turn_speed
        else:
            if sr < ctrl.max_side_range:
                error = sr - ctrl.target_dist
                side = "RIGHT"
            elif sl < ctrl.max_side_range:
                error = ctrl.target_dist - sl
                side = "LEFT"
            else:
                error = 0.0
                side = None

            if side is not None:
                ctrl.integral += error * dt
                ctrl.integral = max(-1.0, min(1.0, ctrl.integral))

                derivative = (error - ctrl.prev_error) / dt if dt > 0 else 0.0
                ctrl.prev_error = error

                steering = (ctrl.Kp * error) + (ctrl.Ki * ctrl.integral) + (ctrl.Kd * derivative)

                if side == "RIGHT":
                    left_vel = ctrl.base_speed + steering
                    right_vel = ctrl.base_speed - steering
                else:
                    left_vel = ctrl.base_speed - steering
                    right_vel = ctrl.base_speed + steering
            else:
                left_vel = ctrl.base_speed
                right_vel = ctrl.base_speed
    # ======================================================================

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