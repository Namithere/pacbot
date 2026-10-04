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


# --- Controller State & Tuning Parameters ---
class ControllerState:
    def __init__(self):
        self.state = "FOLLOW"       # "FOLLOW", "TURN"
        self.turn_direction = None  # "LEFT" or "RIGHT"
        self.accumulated_yaw = 0.0  # Radians rotated during active turn
        self.target_turn_angle = 0.0

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0

        # Setpoints & Thresholds (meters)
        self.target_dist = 0.12     # Desired distance to side wall
        self.front_wall_dist = 0.14 # Distance triggering a turn
        self.max_side_range = 0.30  # Max distance to consider a wall present

        # Speeds (rad/s)
        self.base_speed = 10.0
        self.turn_speed = 6.0

        # PID Gains
        self.Kp = 35.0
        self.Ki = 0.2
        self.Kd = 1.5


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

    print(f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f}")

    # ========================== CONTROLLER LOGIC ==========================
    left_vel = 0.0
    right_vel = 0.0

    front_dist = min(fl, fr)

    if ctrl.state == "TURN":
        # Integrate gyro to track exact angle turned
        ctrl.accumulated_yaw += yaw_rate * dt

        if abs(ctrl.accumulated_yaw) >= ctrl.target_turn_angle:
            # Turn complete, resume wall following
            ctrl.state = "FOLLOW"
            ctrl.accumulated_yaw = 0.0
            ctrl.integral = 0.0
            ctrl.prev_error = 0.0
        else:
            # Execute in-place spin
            if ctrl.turn_direction == "LEFT":
                left_vel = -ctrl.turn_speed
                right_vel = ctrl.turn_speed
            else:
                left_vel = ctrl.turn_speed
                right_vel = -ctrl.turn_speed

    elif ctrl.state == "FOLLOW":
        # Check if approaching a wall ahead
        if front_dist < ctrl.front_wall_dist:
            ctrl.state = "TURN"
            ctrl.accumulated_yaw = 0.0
            ctrl.integral = 0.0
            ctrl.prev_error = 0.0

            # Decide turn direction based on side openings
            if sl > sr and sl > ctrl.target_dist:
                ctrl.turn_direction = "LEFT"
                ctrl.target_turn_angle = math.pi / 2.0
            elif sr > ctrl.target_dist:
                ctrl.turn_direction = "RIGHT"
                ctrl.target_turn_angle = math.pi / 2.0
            else:
                # Dead end: perform a 180-degree turn
                ctrl.turn_direction = "LEFT"
                ctrl.target_turn_angle = math.pi

            # Apply initial turn velocity
            if ctrl.turn_direction == "LEFT":
                left_vel = -ctrl.turn_speed
                right_vel = ctrl.turn_speed
            else:
                left_vel = ctrl.turn_speed
                right_vel = -ctrl.turn_speed

        else:
            # Wall following via PID
            # Prefer tracking the right wall; fall back to left if right is absent
            if sr < ctrl.max_side_range:
                # Error is positive if we are too far from the right wall
                error = sr - ctrl.target_dist
                side = "RIGHT"
            elif sl < ctrl.max_side_range:
                # Error is positive if we are too far from the left wall
                error = ctrl.target_dist - sl
                side = "LEFT"
            else:
                error = 0.0
                side = None

            if side is not None:
                ctrl.integral += error * dt
                # Anti-windup clamping
                ctrl.integral = max(-1.0, min(1.0, ctrl.integral))

                derivative = (error - ctrl.prev_error) / dt if dt > 0 else 0.0
                ctrl.prev_error = error

                # PID steering correction
                steering = (ctrl.Kp * error) + (ctrl.Ki * ctrl.integral) + (ctrl.Kd * derivative)

                if side == "RIGHT":
                    # Steer right if too far from right wall (positive error)
                    left_vel = ctrl.base_speed + steering
                    right_vel = ctrl.base_speed - steering
                else:
                    # Steer left if too far from left wall
                    left_vel = ctrl.base_speed - steering
                    right_vel = ctrl.base_speed + steering
            else:
                # Open area, drive straight
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