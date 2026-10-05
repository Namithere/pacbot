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


# ======================= YOUR CODE: CONTROLLER SETUP =======================
# --- Tunables ---
SWAP_SIDES = True          # True: treat "sl" sensor as right side and "sr" as left
SPEED_SCALE = 1.0          # Realistic speed multiplier
BASE_SPEED = 7.0 * SPEED_SCALE     # rad/s, cruise wheel speed
MAX_WHEEL = 10.0 * SPEED_SCALE    # rad/s, saturation limit
MIN_SPEED = 2.0            # rad/s, creep speed at the end of braking ramp
MAX_RANGE = 2.0            # m, value used for invalid / infinite ToF readings

CORRIDOR_W = 0.22          # m, distance between the two walls
OPEN_THRESH = 0.16         # m, a side reading above this means an open junction/gap

# Tuned deeper into the intersection to avoid premature corner clipping
FRONT_STOP = 0.065         # m, stop distance from front wall
BRAKE_MARGIN = 0.045       # m, start braking at (FRONT_STOP + BRAKE_MARGIN) = 0.11 m
BRAKE_K = 18.0             # rad/s per sqrt(m)
BACKOFF_DIST = 0.035       # m, reverse if pushed too close
BACKOFF_SPEED = 1.5        # rad/s, reverse speed for backing off

STOP_MIN_T = 0.10          # s, minimum time held at zero before a turn may start
STOP_MAX_T = 0.80          # s, give up waiting for "fully stopped" after this long
STILL_WIN = 0.04           # s, window used to check the front distance has stopped changing
STILL_DIST = 0.003         # m, front moved less than this in a window -> counts as still
STILL_YAW = 0.05           # rad/s, gyro rate below this -> not rotating

CENTER_KP, CENTER_KI, CENTER_KD = 90.0, 0.4, 5.0   # centring PID (error in m)
HOLD_KP, HOLD_KI, HOLD_KD = 7.0, 0.0, 0.2          # heading PID (straight-line hold)
STEER_MAX = 8.0            # rad/s, max differential from centring / heading hold

# --- Turning ---
TURN_ANGLE_DEG = 90.0      # deg, size of each turn
TURN_KP, TURN_KI, TURN_KD = 4.5, 0.0, 0.5   # heading PID for in-place turns
TURN_MAX = 4.0             # rad/s, controlled wheel speed during in-place turns
TURN_MIN = 0.8             # rad/s, floor so the final degrees still finish
TURN_SLOW_K = 4.5          # rad/s per sqrt(rad): turn speed falls off near target
TURN_LEAD_T = 0.06         # s, look-ahead prediction window
TURN_TOL = math.radians(2.0)   # rad, heading error considered "done"

PRINT_EVERY = 50           # print one line every N sensor messages (~10 Hz at 500 Hz)


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
_turn_pid = PID(TURN_KP, TURN_KI, TURN_KD, out_limit=TURN_MAX, i_limit=1.0)
_hold_pid = PID(HOLD_KP, HOLD_KI, HOLD_KD, out_limit=STEER_MAX, i_limit=1.0)

# Controller state
_state = {
    "mode": "STRAIGHT",   # STRAIGHT -> STOP -> TURN -> STRAIGHT ...
    "heading": 0.0,       # rad, integrated gyro yaw
    "hold": 0.0,          # rad, heading to hold on straight runs
    "target": 0.0,        # rad, heading goal while TURN
    "turn": 0.0,          # rad, turn chosen when the stop began (+ = left)
    "stop_t": 0.0,        # s, time spent in STOP
    "win_t": 0.0,         # s, time inside the current "is it still?" window
    "win_front": 0.0,     # m, front reading at the start of the window
    "still_n": 0,         # consecutive still windows
    "front_hits": 0,      # consecutive confirmations of front wall
    "count": 0,           # sensor messages seen (for throttled printing)
}


def _clean(x):
    """Replace NaN / inf / dropout readings with MAX_RANGE."""
    try:
        x = float(x)
    except (TypeError, ValueError):
        return MAX_RANGE
    if math.isnan(x) or math.isinf(x):
        return MAX_RANGE
    # Dropouts or uninitialized sensor packets reporting near 0 are treated as clear
    if x <= 0.005:
        return MAX_RANGE
    return min(x, MAX_RANGE)


def _clamp(v):
    return max(-MAX_WHEEL, min(MAX_WHEEL, v))


def _set_mode(new_mode):
    if new_mode != _state["mode"]:
        print(f"[mode] {_state['mode']} -> {new_mode}  "
              f"heading={math.degrees(_state['heading']):+.1f} deg")
        _state["mode"] = new_mode


def _approach_speed(front):
    """Speed allowed at this front distance (constant-deceleration braking curve)."""
    d = max(0.0, front - FRONT_STOP - BRAKE_MARGIN)
    return min(BASE_SPEED, MIN_SPEED + BRAKE_K * math.sqrt(d))


def _hold_straight(speed, dt):
    """Drive at `speed` while holding the stored heading using the gyro."""
    err = _state["hold"] - _state["heading"]
    w = _hold_pid.update(err, dt)               # + = steer left
    return _clamp(speed - w), _clamp(speed + w)


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    """Return (left_vel, right_vel) in rad/s."""
    fl, fr, sl, sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    if SWAP_SIDES:
        sl, sr = sr, sl
    _state["heading"] += yaw_rate * dt

    front = min(fl, fr)
    mode = _state["mode"]

    # ---- STOP: wheels at zero; only turn once the bot settles ----
    if mode == "STOP":
        _state["stop_t"] += dt

        if front < BACKOFF_DIST:
            _state["win_t"], _state["win_front"], _state["still_n"] = 0.0, front, 0
            return -BACKOFF_SPEED, -BACKOFF_SPEED

        _state["win_t"] += dt
        if _state["win_t"] >= STILL_WIN:
            moved = abs(front - _state["win_front"])
            if moved < STILL_DIST and abs(yaw_rate) < STILL_YAW:
                _state["still_n"] += 1
            else:
                _state["still_n"] = 0
            _state["win_t"], _state["win_front"] = 0.0, front

        at_rest = _state["still_n"] >= 2 and _state["stop_t"] >= STOP_MIN_T
        if at_rest or _state["stop_t"] >= STOP_MAX_T:
            _state["target"] = _state["heading"] + _state["turn"]
            _turn_pid.reset()
            _set_mode("TURN")
        return 0.0, 0.0

    # ---- TURN: rotate in place using gyro heading ----
    if mode == "TURN":
        err = _state["target"] - _state["heading"]
        if abs(err) < TURN_TOL and abs(yaw_rate) < 0.25:
            _state["hold"] = _state["target"]
            _hold_pid.reset()
            _center_pid.reset()
            _set_mode("STRAIGHT")
            return 0.0, 0.0
        pred_err = err - yaw_rate * TURN_LEAD_T
        w = _turn_pid.update(pred_err, dt)
        w_max = min(TURN_MAX, TURN_MIN + TURN_SLOW_K * math.sqrt(abs(err)))
        w = max(-w_max, min(w_max, w))
        return _clamp(-w), _clamp(w)

    # ---- STRAIGHT: drive forward, center in corridor, verify front wall ----
    if front < FRONT_STOP:
        _state["front_hits"] += 1
        if _state["front_hits"] >= 2:  # Debounce: requires 2 consecutive readings
            angle = math.radians(TURN_ANGLE_DEG)
            if abs(sl - sr) < 0.025:
                _state["turn"] = angle  # default left if symmetric
            else:
                _state["turn"] = angle if sl > sr else -angle
            _state.update(stop_t=0.0, win_t=0.0, win_front=front, still_n=0, front_hits=0)
            _set_mode("STOP")
            return 0.0, 0.0
    else:
        _state["front_hits"] = 0

    speed = _approach_speed(front)

    left_ok, right_ok = sl < OPEN_THRESH, sr < OPEN_THRESH
    if left_ok and right_ok:
        _state["hold"] = _state["heading"]
        err = 0.5 * (sl - sr)
        u = _center_pid.update(err, dt)
        return _clamp(speed - u), _clamp(speed + u)

    # Wall missing on one or both sides: maintain heading via gyro
    _center_pid.reset()
    return _hold_straight(speed, dt)


def _log(fl, fr, sl, sr, yaw_rate, dt, left_vel, right_vel):
    """Throttled status line."""
    _state["count"] += 1
    if _state["count"] % PRINT_EVERY == 0:
        front = min(_clean(fl), _clean(fr))
        print(f"{_state['mode']:<8} front={front:.3f} sl={_clean(sl):.3f} sr={_clean(sr):.3f} "
              f"cmd L={left_vel:+.1f} R={right_vel:+.1f} "
              f"hdg={math.degrees(_state['heading']):+.1f}deg")
# ===========================================================================


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
    _log(fl, fr, sl, sr, yaw_rate, dt, left_vel, right_vel)

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