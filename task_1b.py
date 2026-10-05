"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs readings, and publishes
wheel velocity commands.

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
# Tunables (Tuned for BASE_SPEED = 100)
# ----------------------------------------------------------------------------
BASE_SPEED = 100.0         # Extreme high cruise speed (rad/s)
MIN_APPROACH_SPEED = 3.0   # Approach floor before stop (rad/s)

# Stopping thresholds (Extended for massive inertia from 100 rad/s)
FRONT_STOP = 0.115         # Stop and turn trigger distance (m)
SLOW_DIST = 0.850          # Start aggressive deceleration well in advance (m)
CONFIRM_FRAMES = 2

# Turn parameters (Closed-loop PD rotation to eliminate drag/overshoot)
TURN_RATE_MAX = 5.0        # Body yaw rate ceiling during turn (rad/s)
TURN_RATE_MIN = 0.6        # Fine precision creep rate (rad/s)
TURN_KP = 6.0              # Proportional turn gain
TURN_KD = 0.35             # Derivative turn damping to prevent dragging/overshooting 90 deg
TURN_TOL = math.radians(1.2)
TURN_ANGLE = math.pi / 2.0 # Exact 90 degrees

# Side Wall Centering & Heading (Scaled down to prevent wobble at 100 rad/s)
K_HEAD = 12.0
K_GYRO = 1.2
U_LIMIT = 20.0             # Steering cap for 100 rad/s cruise

SIDE_SAFE = 0.055
SIDE_CORRIDOR_MAX = 0.180
K_SIDE_PUSH = 35.0
K_CENTERING = 8.0

# ----------------------------------------------------------------------------
# State Machine
# ----------------------------------------------------------------------------
mode = 'DRIVE'
accumulated_yaw = 0.0
target_angle = 0.0
front_hit_counter = 0
brake_timer = 0
settle_timer = 0
log_tick = 0


def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global mode, accumulated_yaw, target_angle
    global front_hit_counter, brake_timer, settle_timer, log_tick

    data = json.loads(msg.payload.decode())

    fl = float(data["fl"])
    fr = float(data["fr"])
    sl = float(data["sl"])
    sr = float(data["sr"])
    yaw_rate = float(data["gyro"][2])
    dt = float(data["dt"]) if data.get("dt") and data["dt"] > 0 else 0.002

    # Robust front distance selection
    valid_fronts = [d for d in (fl, fr) if d > 0.02]
    front_dist = min(valid_fronts) if valid_fronts else 0.5

    left_vel = 0.0
    right_vel = 0.0

    if mode == 'DRIVE':
        accumulated_yaw += yaw_rate * dt

        if front_dist <= FRONT_STOP:
            front_hit_counter += 1
        else:
            front_hit_counter = 0

        if front_hit_counter >= CONFIRM_FRAMES:
            # Slam counter-torque to immediately stop momentum from 100 rad/s
            mode = 'ACTIVE_BRAKE'
            brake_timer = 0
            front_hit_counter = 0
            left_vel = -25.0
            right_vel = -25.0
        else:
            # Aggressive multi-stage braking curve to shed 100 rad/s
            if front_dist < SLOW_DIST:
                ratio = (front_dist - FRONT_STOP) / (SLOW_DIST - FRONT_STOP)
                ratio = max(0.0, min(1.0, ratio))
                speed = MIN_APPROACH_SPEED + (ratio ** 1.8) * (BASE_SPEED - MIN_APPROACH_SPEED)
            else:
                speed = BASE_SPEED

            # Side centering & wall repulsion
            side_correction = 0.0
            if sl < SIDE_CORRIDOR_MAX and sr < SIDE_CORRIDOR_MAX:
                side_correction = K_CENTERING * (sl - sr)

            if sl < SIDE_SAFE:
                side_correction -= K_SIDE_PUSH * (SIDE_SAFE - sl)
                accumulated_yaw = 0.0
            elif sr < SIDE_SAFE:
                side_correction += K_SIDE_PUSH * (SIDE_SAFE - sr)
                accumulated_yaw = 0.0

            # Heading hold combined with centering
            heading_hold = (-K_HEAD * accumulated_yaw) - (K_GYRO * yaw_rate)
            steering = heading_hold + side_correction
            steering = max(-U_LIMIT, min(U_LIMIT, steering))

            left_vel = speed - steering
            right_vel = speed + steering

    elif mode == 'ACTIVE_BRAKE':
        # Apply reverse pulse for ~30ms to fully neutralize skid
        left_vel = -25.0
        right_vel = -25.0
        brake_timer += 1

        if brake_timer >= 12:
            mode = 'SAMPLE_AND_DECIDE'
            settle_timer = 0
            left_vel = 0.0
            right_vel = 0.0

    elif mode == 'SAMPLE_AND_DECIDE':
        # Zero velocity pause to let chassis settle and read true clearance
        left_vel = 0.0
        right_vel = 0.0
        settle_timer += 1

        if settle_timer >= 10:
            accumulated_yaw = 0.0
            if sl >= sr:
                target_angle = TURN_ANGLE      # +90 deg
            else:
                target_angle = -TURN_ANGLE     # -90 deg
            mode = 'TURN'

    elif mode == 'TURN':
        accumulated_yaw += yaw_rate * dt
        err = target_angle - accumulated_yaw

        # PD control on rotation to damp out drag and land on exactly 90 degrees
        w_cmd = (TURN_KP * err) - (TURN_KD * yaw_rate)

        if abs(err) <= TURN_TOL and abs(yaw_rate) < 0.2:
            # Exactly landed on 90 degrees with zero residual spin
            accumulated_yaw = 0.0
            mode = 'DRIVE'
            front_hit_counter = 0
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED
        else:
            w_clamped = math.copysign(
                max(TURN_RATE_MIN, min(TURN_RATE_MAX, abs(w_cmd))),
                w_cmd
            )
            wheel_speed = w_clamped * (TRACK / 2.0) / WHEEL_R
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