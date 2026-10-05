"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back.
"""
import json
import math
import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ======================= CONTROLLER CONFIG =======================
SWAP_SIDES = True          # Simulator left/right sensor reversal

BASE_SPEED = 5.0           # rad/s constant forward cruise
MAX_WHEEL = 8.0            # rad/s wheel saturation
TURN_SPEED = 3.5           # rad/s constant spin speed during 90-deg turn

CORRIDOR_W = 0.22          # m
FRONT_STOP = 0.075         # m, stop distance from front wall
SIDE_WALL_THRESH = 0.16    # m, values larger than this mean an opening

TURN_ANGLE = math.radians(90.0)
TURN_TOL = math.radians(4.0)  # 4 degrees tolerance to exit turn cleanly

# Simple state tracking
_state = {
    "mode": "STRAIGHT",     # "STRAIGHT", "STOP", "TURN"
    "heading": 0.0,
    "target_heading": 0.0,
    "stop_timer": 0.0,
    "front_confirm": 0,
    "count": 0,
}


def _clean(x):
    """Filter invalid readings."""
    try:
        val = float(x)
        if math.isnan(val) or math.isinf(val) or val <= 0.005:
            return 2.0
        return min(val, 2.0)
    except (TypeError, ValueError):
        return 2.0


def _clamp(v):
    return max(-MAX_WHEEL, min(MAX_WHEEL, v))


def _set_mode(new_mode):
    if new_mode != _state["mode"]:
        print(f"[mode] {_state['mode']} -> {new_mode} | Hdg: {math.degrees(_state['heading']):.1f}°")
        _state["mode"] = new_mode


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    fl, fr = _clean(fl), _clean(fr)
    sl, sr = _clean(sl), _clean(sr)
    if SWAP_SIDES:
        sl, sr = sr, sl

    _state["heading"] += yaw_rate * dt
    front = min(fl, fr)
    mode = _state["mode"]

    # ---------------- 1. STOP MODE (Pause briefly before turning) ----------------
    if mode == "STOP":
        _state["stop_timer"] += dt
        if _state["stop_timer"] >= 0.15:  # Pause for 150 ms to kill linear momentum
            _set_mode("TURN")
        return 0.0, 0.0

    # ---------------- 2. TURN MODE (Rotate precisely by 90 deg) ----------------
    if mode == "TURN":
        err = _state["target_heading"] - _state["heading"]

        # Turn finished when target angle is reached within tolerance
        if abs(err) <= TURN_TOL:
            _set_mode("STRAIGHT")
            return 0.0, 0.0

        # Rotate left (+err) or right (-err)
        direction = 1.0 if err > 0 else -1.0
        left_cmd = -direction * TURN_SPEED
        right_cmd = direction * TURN_SPEED
        return _clamp(left_cmd), _clamp(right_cmd)

    # ---------------- 3. STRAIGHT MODE (Drive & Center) ----------------
    # Check if wall is reached
    if front <= FRONT_STOP:
        _state["front_confirm"] += 1
        if _state["front_confirm"] >= 2:
            # Wall detected: choose turn direction towards the opening
            # sl > sr means the left has more free space -> turn left (+90 deg)
            if sl > sr:
                chosen_turn = TURN_ANGLE
            else:
                chosen_turn = -TURN_ANGLE

            _state["target_heading"] = _state["heading"] + chosen_turn
            _state["stop_timer"] = 0.0
            _state["front_confirm"] = 0
            _set_mode("STOP")
            return 0.0, 0.0
    else:
        _state["front_confirm"] = 0

    # Straight line steering: center between walls if both present
    steer = 0.0
    if sl < SIDE_WALL_THRESH and sr < SIDE_WALL_THRESH:
        # P-controller on distance difference to stay centered
        err_center = sl - sr  # positive if closer to right wall
        steer = 15.0 * err_center

    left_cmd = BASE_SPEED - steer
    right_cmd = BASE_SPEED + steer
    return _clamp(left_cmd), _clamp(right_cmd)


def _log(fl, fr, sl, sr, yaw_rate, left_vel, right_vel):
    _state["count"] += 1
    if _state["count"] % 40 == 0:
        front = min(_clean(fl), _clean(fr))
        print(f"[{_state['mode']}] Front: {front:.3f}m | Left: {_clean(sl):.3f}m | Right: {_clean(sr):.3f}m | "
              f"L_cmd: {left_vel:+.1f} R_cmd: {right_vel:+.1f} | Hdg: {math.degrees(_state['heading']):.1f}°")


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

    left_vel, right_vel = _controller(fl, fr, sl, sr, yaw_rate, dt)
    _log(fl, fr, sl, sr, yaw_rate, left_vel, right_vel)

    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel),
        "right": float(right_cmd if 'right_cmd' in locals() else right_vel)
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