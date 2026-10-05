"""
Complete PB Task 1B Controller.

Subscribes to pacbot/sensors, drives forward with proportional wall centering,
decelerates smoothly, and pivots immediately in place when reaching FRONT_STOP (0.09m).

Run in separate terminals:
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py
"""

import json
import math
import paho.mqtt.client as mqtt

# ----------------- MQTT BROKER SETTINGS -----------------
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ======================= CONTROLLER SETUP =======================
SWAP_SIDES = True          # True: map "sl" as right, "sr" as left

# --- Speed & Saturation Limits ---
BASE_SPEED = 14.0          # Safe, fast cruising velocity (rad/s)
MAX_WHEEL = 25.0           # Saturation ceiling for steering adjustments
MIN_SPEED = 4.0            # Creep speed when closing in on a wall
MAX_RANGE = 2.0            # Default value for ray misses or invalid ToF

# --- Wall Detection & Braking ---
FRONT_STOP = 0.09          # Desired turn threshold (9 cm)
BRAKE_DIST = 0.20          # Start slowing down proportionally at 20 cm

# --- Fast In-Place Turning ---
TURN_ANGLE_DEG = 90.0
TURN_SPEED = 8.5           # Wheel speed during in-place spin (rad/s)
TURN_TOL = math.radians(3.5)

# --- Corridor Centering ---
CORRIDOR_W = 0.22
OPEN_THRESH = 0.18
CENTER_KP = 75.0           # Proportional centering gain
CENTER_KD = 8.0            # Derivative gain to damp oscillation
HOLD_KP = 8.0
HOLD_KD = 0.4
STEER_MAX = 18.0           # Steering differential limit

PRINT_EVERY = 50           # Throttled console logging interval


class PID:
    def __init__(self, kp, ki, kd, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.reset()

    def reset(self):
        self.prev_err = None

    def update(self, err, dt):
        if dt <= 0.0:
            return 0.0
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.kd * deriv
        return max(-self.out_limit, min(self.out_limit, out))


_center_pid = PID(CENTER_KP, 0.0, CENTER_KD, out_limit=STEER_MAX)
_hold_pid = PID(HOLD_KP, 0.0, HOLD_KD, out_limit=STEER_MAX)

_state = {
    "mode": "DRIVE",
    "heading": 0.0,
    "hold": 0.0,
    "target": 0.0,
    "turn": 0.0,
    "total_t": 0.0,        # Startup guard to prevent t=0 false triggers
    "count": 0,
}


def _clean(x):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return MAX_RANGE
    if math.isnan(x) or math.isinf(x) or x < 0.0:
        return MAX_RANGE
    return min(x, MAX_RANGE)


def _clamp(v):
    return max(-MAX_WHEEL, min(MAX_WHEEL, v))


def _set_mode(new_mode):
    if new_mode != _state["mode"]:
        print(f"[MODE] {_state['mode']} -> {new_mode} | Heading: {math.degrees(_state['heading']):+.1f}°")
        _state["mode"] = new_mode


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    fl, fr, sl, sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    if SWAP_SIDES:
        sl, sr = sr, sl

    _state["heading"] += yaw_rate * dt
    _state["total_t"] += dt
    front = min(fl, fr)
    mode = _state["mode"]

    # ---------------- 1. DRIVE MODE ----------------
    if mode == "DRIVE":
        # Ignore false wall hits in the first 0.15s of simulation initialization
        if front <= FRONT_STOP and _state["total_t"] > 0.15:
            angle = math.radians(TURN_ANGLE_DEG)
            # Turn toward whichever side has more clearance
            turn_dir = 1.0 if sl >= sr else -1.0
            _state["turn"] = turn_dir * angle
            _state["target"] = _state["heading"] + _state["turn"]

            print(f"[WALL DETECTED] front={front:.3f}m | sl={sl:.3f}m sr={sr:.3f}m -> Turning {'LEFT' if turn_dir > 0 else 'RIGHT'}")
            _set_mode("TURN")
            # Apply active differential spin immediately
            return _clamp(-turn_dir * TURN_SPEED), _clamp(turn_dir * TURN_SPEED)

        # Proportional speed reduction between BRAKE_DIST and FRONT_STOP
        if front < BRAKE_DIST:
            ratio = max(0.0, (front - FRONT_STOP) / (BRAKE_DIST - FRONT_STOP))
            speed = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * ratio
        else:
            speed = BASE_SPEED

        # Corridor centering: walls present on both sides
        if sl < OPEN_THRESH and sr < OPEN_THRESH:
            _state["hold"] = _state["heading"]
            u = _center_pid.update(0.5 * (sl - sr), dt)
            return _clamp(speed - u), _clamp(speed + u)

        # One or both sides open: lock gyro heading straight
        _center_pid.reset()
        w = _hold_pid.update(_state["hold"] - _state["heading"], dt)
        return _clamp(speed - w), _clamp(speed + w)

    # ---------------- 2. TURN MODE ----------------
    if mode == "TURN":
        err = _state["target"] - _state["heading"]

        # Turn completion check
        if abs(err) < TURN_TOL:
            print(f"[TURN DONE] Exiting turn at {math.degrees(_state['heading']):.1f}°")
            _state["hold"] = _state["target"]
            _center_pid.reset()
            _hold_pid.reset()
            _set_mode("DRIVE")
            return 0.0, 0.0

        # Pivot in place until heading matches target
        direction = math.copysign(1.0, err)
        return _clamp(-direction * TURN_SPEED), _clamp(direction * TURN_SPEED)

    return 0.0, 0.0


def _log(fl, fr, sl, sr, yaw_rate, dt, left_vel, right_vel):
    _state["count"] += 1
    if _state["count"] % PRINT_EVERY == 0:
        front = min(_clean(fl), _clean(fr))
        print(f"{_state['mode']:<5} front={front:.3f} sl={_clean(sl):.3f} sr={_clean(sr):.3f} "
              f"L={left_vel:+.1f} R={right_vel:+.1f} hdg={math.degrees(_state['heading']):+.1f}°")


def _mqtt_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    yaw_rate = data["gyro"][2]
    dt = data["dt"]

    left_vel, right_vel = _controller(
        data["fl"], data["fr"], data["sl"], data["sr"], yaw_rate, dt
    )

    _log(data["fl"], data["fr"], data["sl"], data["sr"], yaw_rate, dt, left_vel, right_vel)

    # Fast direct JSON string formatting
    payload = f'{{"left":{float(left_vel):.3f},"right":{float(right_vel):.3f}}}'
    client.publish(TOPIC_WHEEL_VEL, payload, qos=0)


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS, qos=0)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass