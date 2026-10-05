"""Boilerplate for PB Task 1B with active sensor logging and wall autocorrection.

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
# Tunables
# ----------------------------------------------------------------------------
BASE_SPEED = 10.0          # Controlled high speed (rad/s)
MIN_APPROACH_SPEED = 1.5   # Creep floor near walls (rad/s)

# Distance thresholds
FRONT_STOP = 0.092         # Confirmed stopping distance (m)
SLOW_DIST = 0.350          # Deceleration start distance (m)
CONFIRM_FRAMES = 2         # Consecutive frames below FRONT_STOP

# Side Wall Autocorrection & Centering
SIDE_SAFE = 0.055          # Minimum safe distance from any side wall (m)
SIDE_CORRIDOR_MAX = 0.180  # Max distance to consider a wall present on that side (m)
K_SIDE_PUSH = 45.0         # Aggressive repulsion when closer than SIDE_SAFE
K_CENTERING = 12.0         # Proportional centering gain when both walls exist

# Turn parameters
TURN_RATE_MAX = 4.5        # Fast in-place rotation body rate (rad/s)
TURN_RATE_MIN = 1.0        # Landing crawl rate (rad/s)
TURN_KP = 6.0              # Proportional gain
TURN_TOL = math.radians(2.0)
TURN_ANGLE = math.pi / 2.0 # 90 degrees

# Heading hold gains
K_HEAD = 4.0
K_GYRO = 0.7
U_LIMIT = 3.0

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

    front_dist = min(fl, fr)
    left_vel = 0.0
    right_vel = 0.0

    if mode == 'DRIVE':
        accumulated_yaw += yaw_rate * dt

        if front_dist <= FRONT_STOP:
            front_hit_counter += 1
        else:
            front_hit_counter = 0

        if front_hit_counter >= CONFIRM_FRAMES:
            mode = 'ACTIVE_BRAKE'
            brake_timer = 0
            front_hit_counter = 0
            left_vel = -3.5
            right_vel = -3.5
        else:
            if front_dist < SLOW_DIST:
                ratio = (front_dist - FRONT_STOP) / (SLOW_DIST - FRONT_STOP)
                ratio = max(0.0, min(1.0, ratio))
                speed = MIN_APPROACH_SPEED + (ratio ** 1.3) * (BASE_SPEED - MIN_APPROACH_SPEED)
            else:
                speed = BASE_SPEED

            # --- Side Wall Autocorrection Logic ---
            side_correction = 0.0

            # 1. Proportional Centering when both walls are detected
            if sl < SIDE_CORRIDOR_MAX and sr < SIDE_CORRIDOR_MAX:
                centering_error = sl - sr  # positive means robot is shifted toward right wall
                side_correction = K_CENTERING * centering_error

            # 2. Emergency Repulsion if dangerously close to either wall
            if sl < SIDE_SAFE:
                side_correction -= K_SIDE_PUSH * (SIDE_SAFE - sl)  # steer right
                accumulated_yaw = 0.0
            elif sr < SIDE_SAFE:
                side_correction += K_SIDE_PUSH * (SIDE_SAFE - sr)  # steer left
                accumulated_yaw = 0.0

            # Heading hold combined with wall autocorrection
            heading_hold = (-K_HEAD * accumulated_yaw) - (K_GYRO * yaw_rate)
            steering = heading_hold + side_correction
            steering = max(-U_LIMIT, min(U_LIMIT, steering))

            left_vel = speed - steering
            right_vel = speed + steering

    elif mode == 'ACTIVE_BRAKE':
        left_vel = -3.5
        right_vel = -3.5
        brake_timer += 1

        if brake_timer >= 6:
            mode = 'SAMPLE_AND_DECIDE'
            settle_timer = 0
            left_vel = 0.0
            right_vel = 0.0

    elif mode == 'SAMPLE_AND_DECIDE':
        left_vel = 0.0
        right_vel = 0.0
        settle_timer += 1

        if settle_timer >= 6:
            accumulated_yaw = 0.0
            if sl >= sr:
                target_angle = TURN_ANGLE
            else:
                target_angle = -TURN_ANGLE
            mode = 'TURN'

    elif mode == 'TURN':
        accumulated_yaw += yaw_rate * dt
        err = target_angle - accumulated_yaw

        if abs(err) <= TURN_TOL:
            accumulated_yaw = 0.0
            mode = 'DRIVE'
            front_hit_counter = 0
            left_vel = BASE_SPEED
            right_vel = BASE_SPEED
        else:
            w = math.copysign(
                max(TURN_RATE_MIN, min(TURN_RATE_MAX, TURN_KP * abs(err))),
                err
            )
            wheel_speed = w * (TRACK / 2.0) / WHEEL_R
            left_vel = -wheel_speed
            right_vel = wheel_speed

    # Print log every 20 ticks (~25Hz)
    log_tick += 1
    if log_tick % 20 == 0:
        print(f"[{mode:17s}] front={front_dist:.3f} | fl={fl:.3f} fr={fr:.3f} | "
              f"sl={sl:.3f} sr={sr:.3f} | yaw={math.degrees(accumulated_yaw):+6.1f}° | "
              f"L={left_vel:+5.1f} R={right_vel:+5.1f}")

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