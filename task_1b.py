"""
Optimized PB Task 1B Controller - Crash-Proofed High-Speed Version
"""

import json
import math
import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ======================= CONTROLLER SETUP =======================
SWAP_SIDES = True          # True: map "sl" as right, "sr" as left

# --- Tunables ---
BASE_SPEED = 22.0          # Controlled high forward speed (rad/s)
MAX_WHEEL = 35.0           # Saturation ceiling for steering adjustments
MIN_SPEED = 5.0            # Creep speed approaching wall
MAX_RANGE = 2.0            # Default for invalid ToF readings

# --- Wall in front -> stop -> turn ---
# Increased to give adequate braking distance at ~20 rad/s
FRONT_STOP = 0.11          # m, start turn routine well before mechanical collision
BACKOFF_DIST = 0.05        # m, emergency reverse threshold
BACKOFF_SPEED = 6.0        # rad/s, reverse speed
BRAKE_MARGIN = 0.12        # m, begins deceleration ramp well in advance
BRAKE_K = 35.0             # Braking curvature: slows smoothly to MIN_SPEED
A_DECEL = 6.0              # Deceleration ceiling for crash guard
VEL_WIN = 0.02             # Window for closing speed calculation (s)
BRAKE_REV = 20.0           # Reverse command during emergency braking

# --- Turn Timers ---
STOP_MIN_T = 0.01          # Minimum pause before turning (s)
STOP_MAX_T = 0.10          # Cut long timeouts to snap turns immediately
STILL_WIN = 0.02           # Sampling duration
STILL_DIST = 0.008         # m
STILL_YAW = 0.12           # rad/s

# --- Driving between walls ---
CORRIDOR_W = 0.22
OPEN_THRESH = 0.20
CENTER_KP = 85.0           # Centering P-gain
CENTER_KI = 0.0            # Ki=0 to avoid integral windup at high speeds
CENTER_KD = 9.0            # D-gain to kill oscillations
HOLD_KP = 10.0
HOLD_KI = 0.0
HOLD_KD = 0.5
STEER_MAX = 20.0           # Steering differential limit

# --- Fast In-Place Turning ---
TURN_ANGLE_DEG = 90.0
TURN_MAX = 14.0            # Maximum wheel velocity during turn (rad/s)
TURN_YAW_MAX = 25.0        # Angular velocity limit
TURN_DECEL = 20.0          # Rotational deceleration
TURN_MARGIN_DEG = 4.0
TURN_CREEP = 1.0
TURN_YAW_KP = 10.0
TURN_TOL = math.radians(4.0)

PRINT_EVERY = 50


class PID:
    def __init__(self, kp, ki, kd, out_limit, i_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit if i_limit is not None else out_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = None

    def update(self, err, dt):
        if dt <= 0.0:
            return 0.0
        self.integral += err * dt
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        deriv = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
        return max(-self.out_limit, min(self.out_limit, out))


_center_pid = PID(CENTER_KP, CENTER_KI, CENTER_KD, out_limit=STEER_MAX, i_limit=0.5)
_hold_pid = PID(HOLD_KP, HOLD_KI, HOLD_KD, out_limit=STEER_MAX, i_limit=1.0)

_state = {
    "mode": "DRIVE",
    "heading": 0.0,
    "hold": 0.0,
    "target": 0.0,
    "turn": 0.0,
    "stop_t": 0.0,
    "win_t": 0.0,
    "win_front": 0.0,
    "still_n": 0,
    "v_t": 0.0,
    "v_front": None,
    "v_close": 0.0,
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
        print(f"[mode] {_state['mode']} -> {new_mode} "
              f"heading={math.degrees(_state['heading']):+.1f} deg")
        _state["mode"] = new_mode


def _update_closing_speed(front, dt):
    if _state["v_front"] is None:
        _state["v_front"] = front
    _state["v_t"] += dt
    if _state["v_t"] >= VEL_WIN:
        raw = (_state["v_front"] - front) / _state["v_t"]
        _state["v_close"] = 0.5 * _state["v_close"] + 0.5 * raw
        _state["v_front"], _state["v_t"] = front, 0.0


def _approach_speed(front):
    # Smooth deceleration ramp starting BRAKE_MARGIN before FRONT_STOP
    dist_remaining = max(0.0, front - FRONT_STOP)
    if dist_remaining > BRAKE_MARGIN:
        return BASE_SPEED
    return min(BASE_SPEED, MIN_SPEED + BRAKE_K * math.sqrt(dist_remaining))


def _turn_command(err, yaw_rate):
    margin = math.radians(TURN_MARGIN_DEG)
    if abs(err) < TURN_TOL:
        w_des = 0.0
    else:
        speed = math.sqrt(2.0 * TURN_DECEL * max(0.0, abs(err) - margin))
        w_des = math.copysign(min(TURN_YAW_MAX, speed + TURN_CREEP), err)
    w = TURN_YAW_KP * (w_des - yaw_rate)
    w = max(-TURN_MAX, min(TURN_MAX, w))
    return _clamp(-w), _clamp(w)


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    fl, fr, sl, sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    if SWAP_SIDES:
        sl, sr = sr, sl
    _state["heading"] += yaw_rate * dt

    front = min(fl, fr)
    _update_closing_speed(front, dt)
    mode = _state["mode"]

    # ---- STOP: Settle and immediately transition to turn ----
    if mode == "STOP":
        _state["stop_t"] += dt

        if front < BACKOFF_DIST:
            return -BACKOFF_SPEED, -BACKOFF_SPEED

        _state["win_t"] += dt
        if _state["win_t"] >= STILL_WIN:
            moved = abs(front - _state["win_front"])
            if moved < STILL_DIST and abs(yaw_rate) < STILL_YAW:
                _state["still_n"] += 1
            else:
                _state["still_n"] = 0
            _state["win_t"], _state["win_front"] = 0.0, front

        # Transition quickly to eliminate dead time
        if (_state["still_n"] >= 1 and _state["stop_t"] >= STOP_MIN_T) or _state["stop_t"] >= STOP_MAX_T:
            _state["target"] = _state["heading"] + _state["turn"]
            _set_mode("TURN")
        return 0.0, 0.0

    # ---- TURN: In-place pivot toward open corridor ----
    if mode == "TURN":
        err = _state["target"] - _state["heading"]
        if abs(err) < TURN_TOL and abs(yaw_rate) < 0.8:
            _state["hold"] = _state["target"]
            _hold_pid.reset()
            _center_pid.reset()
            _state["v_front"], _state["v_close"] = None, 0.0
            _set_mode("DRIVE")
            return 0.0, 0.0
        return _turn_command(err, yaw_rate)

    # ---- DRIVE: Forward run with proactive front wall trigger ----
    if front < FRONT_STOP:
        angle = math.radians(TURN_ANGLE_DEG)
        _state["turn"] = angle if sl >= sr else -angle
        _state.update(stop_t=0.0, win_t=0.0, win_front=front, still_n=0)
        _set_mode("STOP")
        # Apply active counter-torque immediately to stop forward momentum
        return -BACKOFF_SPEED, -BACKOFF_SPEED

    # Active crash guard: If speed toward wall is too high, actively brake
    v_safe = math.sqrt(2.0 * A_DECEL * max(0.0, front - FRONT_STOP))
    if _state["v_close"] > v_safe:
        return -BRAKE_REV, -BRAKE_REV

    speed = _approach_speed(front)

    if sl < OPEN_THRESH and sr < OPEN_THRESH:
        _state["hold"] = _state["heading"]
        u = _center_pid.update(0.5 * (sl - sr), dt)
        return _clamp(speed - u), _clamp(speed + u)

    _center_pid.reset()
    w = _hold_pid.update(_state["hold"] - _state["heading"], dt)
    return _clamp(speed - w), _clamp(speed + w)


def _log(fl, fr, sl, sr, yaw_rate, dt, left_vel, right_vel):
    _state["count"] += 1
    if _state["count"] % PRINT_EVERY == 0:
        front = min(_clean(fl), _clean(fr))
        print(f"{_state['mode']:<6} front={front:.3f} sl={_clean(sl):.3f} sr={_clean(sr):.3f} "
              f"v={_state['v_close']:+.2f}m/s yaw={yaw_rate:+.2f}rad/s "
              f"cmd L={left_vel:+.1f} R={right_vel:+.1f} "
              f"hdg={math.degrees(_state['heading']):+.1f}deg")


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