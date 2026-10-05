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
# --- Tunables (adjust after watching a few runs) ---
SWAP_SIDES = True          # True: treat the "sl" sensor as the right side and "sr" as the left
SPEED_SCALE = 100.0        # overall speed multiplier (was 10.0, now x10)
BASE_SPEED = 10.0 * SPEED_SCALE    # rad/s, top commanded wheel speed
MAX_WHEEL = 16.0 * SPEED_SCALE     # rad/s, saturation limit
MIN_SPEED = 4.0            # rad/s, creep speed at the end of the braking ramp
MAX_RANGE = 2.0            # m, value used for invalid / infinite ToF readings

CORRIDOR_W = 0.22          # m, distance between the two walls
OPEN_THRESH = 0.20         # m, a side reading above this means no wall on that side
FRONT_STOP = 0.06          # m, front wall closer than this -> stop, then turn
BRAKE_MARGIN = 0.04        # m, braking ramp reaches MIN_SPEED this far before FRONT_STOP
BRAKE_K = 100.0            # rad/s per sqrt(m): allowed speed = MIN_SPEED + K*sqrt(distance)
BACKOFF_DIST = 0.045       # m, if the bot overshoots closer than this, it reverses gently
BACKOFF_SPEED = 3.0        # rad/s, reverse speed used for backing off

STOP_MIN_T = 0.10          # s, minimum time held at zero before a turn may start
STOP_MAX_T = 1.00          # s, give up waiting for "fully stopped" after this long
STILL_WIN = 0.05           # s, window used to check the front distance has stopped changing
STILL_DIST = 0.003         # m, front moved less than this in a window -> counts as still
STILL_YAW = 0.05           # rad/s, gyro rate below this -> not rotating

CENTER_KP, CENTER_KI, CENTER_KD = 120.0, 0.5, 8.0   # centring PID (error in m)
HOLD_KP, HOLD_KI, HOLD_KD = 8.0, 0.0, 0.2           # heading PID (straight-line hold)
STEER_MAX = 20.0           # rad/s, max differential from centring / heading hold

# --- Turning (tuned to stop overshoot) ---
TURN_ANGLE_DEG = 90.0      # deg, size of each turn. If it still ends up past 90, try 85.
TURN_KP, TURN_KI, TURN_KD = 6.0, 0.0, 0.8   # heading PID (turns): more damping than before
TURN_MAX = 6.0             # rad/s, top wheel speed during in-place turns (was 8)
TURN_MIN = 0.8             # rad/s, floor so the final degrees still finish
TURN_SLOW_K = 6.0          # rad/s per sqrt(rad): turn speed falls off near the target
TURN_LEAD_T = 0.06         # s, look-ahead: brake for where the bot WILL be in this long
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

# Controller state (module-level so on_message's signature stays untouched)
_state = {
    "mode": "STRAIGHT", # STRAIGHT -> STOP -> TURN -> STRAIGHT ...
    "heading": 0.0,     # rad, integrated gyro yaw
    "hold": 0.0,        # rad, heading to hold on straight runs
    "target": 0.0,      # rad, heading goal while TURN
    "turn": 0.0,        # rad, turn chosen when the stop began (+ = left)
    "stop_t": 0.0,      # s, time spent in STOP
    "win_t": 0.0,       # s, time inside the current "is it still?" window
    "win_front": 0.0,   # m, front reading at the start of the window
    "still_n": 0,       # consecutive still windows
    "count": 0,         # sensor messages seen (for throttled printing)
}


def _clean(x):
    """Replace NaN / inf / negative ToF values with MAX_RANGE."""
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
        print(f"[mode] {_state['mode']} -> {new_mode}  "
              f"heading={math.degrees(_state['heading']):+.1f} deg")
        _state["mode"] = new_mode


def _approach_speed(front):
    """Speed allowed at this front distance (constant-deceleration braking curve).

    allowed = MIN_SPEED + BRAKE_K * sqrt(distance left before the stop point),
    capped at BASE_SPEED. So the bot is already at creep speed when it reaches
    FRONT_STOP + BRAKE_MARGIN, however high SPEED_SCALE is.
    """
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
    if SWAP_SIDES:                              # fix for side sensors mounted the other way round
        sl, sr = sr, sl
    _state["heading"] += yaw_rate * dt          # gyro-integrated heading

    front = min(fl, fr)
    mode = _state["mode"]

    # ---- STOP: wheels at zero; only turn once the bot is really at rest ----
    if mode == "STOP":
        _state["stop_t"] += dt

        if front < BACKOFF_DIST:                # overshot the stop point -> back off gently
            _state["win_t"], _state["win_front"], _state["still_n"] = 0.0, front, 0
            return -BACKOFF_SPEED, -BACKOFF_SPEED

        _state["win_t"] += dt
        if _state["win_t"] >= STILL_WIN:        # has the front reading stopped changing?
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

    # ---- TURN: rotate in place using gyro heading, slowing down as the target nears ----
    if mode == "TURN":
        err = _state["target"] - _state["heading"]
        if abs(err) < TURN_TOL and abs(yaw_rate) < 0.3:
            _state["hold"] = _state["target"]   # new straight-line reference
            _hold_pid.reset()
            _center_pid.reset()
            _set_mode("STRAIGHT")               # resume driving forward
            return 0.0, 0.0
        pred_err = err - yaw_rate * TURN_LEAD_T         # where the error will be shortly
        w = _turn_pid.update(pred_err, dt)              # + = turn left
        w_max = min(TURN_MAX, TURN_MIN + TURN_SLOW_K * math.sqrt(abs(err)))
        w = max(-w_max, min(w_max, w))                  # slow near the target
        return _clamp(-w), _clamp(w)

    # ---- STRAIGHT: drive until the wall is FRONT_STOP away, then stop (and turn) ----
    if front < FRONT_STOP:
        angle = math.radians(TURN_ANGLE_DEG)
        _state["turn"] = angle if sl >= sr else -angle               # toward the open side
        _state.update(stop_t=0.0, win_t=0.0, win_front=front, still_n=0)
        _set_mode("STOP")
        return 0.0, 0.0

    speed = _approach_speed(front)              # brakes automatically near walls

    left_ok, right_ok = sl < OPEN_THRESH, sr < OPEN_THRESH
    if left_ok and right_ok:
        # Walls on both sides: stay centred in the 0.22 m corridor.
        _state["hold"] = _state["heading"]      # keep hold-heading current for hand-over
        err = 0.5 * (sl - sr)                   # + = closer to the right wall
        u = _center_pid.update(err, dt)         # + = steer left
        return _clamp(speed - u), _clamp(speed + u)

    # Gap on one (or both) sides: ignore it, hold the gyro heading straight past it.
    _center_pid.reset()
    return _hold_straight(speed, dt)


def _log(fl, fr, sl, sr, yaw_rate, dt, left_vel, right_vel):
    """Throttled status line (one per PRINT_EVERY sensor messages)."""
    _state["count"] += 1
    if _state["count"] % PRINT_EVERY == 0:
        front = min(_clean(fl), _clean(fr))
        print(f"{_state['mode']:<8} front={front:.3f} sl={_clean(sl):.3f} sr={_clean(sr):.3f} "
              f"cmd L={left_vel:+.1f} R={right_vel:+.1f} "
              f"hdg={math.degrees(_state['heading']):+.1f}deg")
# ===========================================================================


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

    # Compute wheel velocities (rad/s) from the readings above.
    left_vel, right_vel = _controller(fl, fr, sl, sr, yaw_rate, dt)

    # Printing is throttled to ~10 Hz (and on every mode change) so it doesn't slow the loop.
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