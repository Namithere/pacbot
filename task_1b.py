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

SPEED_SCALE = 10000.0      # forward speed multiplier
_S = SPEED_SCALE / 100.0   # factor relative to the earlier version (100.0 -> 1.0)
_G = math.sqrt(_S)         # steering gains scale with sqrt(speed)
TURN_SCALE = 100.0         # wheel-speed limit multiplier while turning

BASE_SPEED = 10.0 * SPEED_SCALE    # rad/s, top commanded wheel speed
MAX_WHEEL = 16.0 * SPEED_SCALE     # rad/s, saturation limit
MIN_SPEED = 4.0            # rad/s, creep speed when reaching the wall
MAX_RANGE = 2.0            # m, value used for invalid / infinite ToF readings

# --- Wall in front -> stop -> turn ---
FRONT_STOP = 0.045         # m, front wall closer than this -> stop, then turn
BACKOFF_DIST = 0.030       # m, closer than this (must be < FRONT_STOP) -> reverse gently
BACKOFF_SPEED = 3.0        # rad/s, reverse speed used for backing off
BRAKE_MARGIN = 0.04 * _G   # m, braking ramp reaches MIN_SPEED this far before FRONT_STOP
BRAKE_K = 100.0 * _S       # rad/s per sqrt(m): allowed speed = MIN_SPEED + K*sqrt(distance)
A_DECEL = 4.0              # m/s^2, braking trusted for the crash guard. Lower = safer.
VEL_WIN = 0.02             # s, window used to measure closing speed from the front ToF
BRAKE_REV = 20.0           # rad/s, reverse command when the crash guard brakes hard

STOP_MIN_T = 0.10          # s, minimum time held at zero before turning
STOP_MAX_T = 1.00          # s, stop waiting for "fully stopped" after this long
STILL_WIN = 0.05           # s, window used to check the front distance stopped changing
STILL_DIST = 0.003         # m, front moved less than this in a window -> still
STILL_YAW = 0.05           # rad/s, gyro rate below this -> not rotating

# --- Driving between walls ---
CORRIDOR_W = 0.22          # m, distance between the two walls
OPEN_THRESH = 0.20         # m, side reading above this -> no wall on that side
CENTER_KP, CENTER_KI, CENTER_KD = 120.0 * _G, 0.5, 8.0 * _G   # centring PID (error in m)
HOLD_KP, HOLD_KI, HOLD_KD = 8.0 * _G, 0.0, 0.2 * _G           # heading-hold PID (rad)
STEER_MAX = 20.0 * _G      # rad/s, max wheel differential while driving

# --- Turning (in place, tracked on the measured gyro yaw rate) ---
TURN_ANGLE_DEG = 90.0      # deg, size of each turn
TURN_MAX = 6.0 * TURN_SCALE    # rad/s, wheel-speed limit during turns
TURN_YAW_MAX = 20.0        # rad/s, fastest body spin allowed
TURN_DECEL = 15.0          # rad/s^2, spin-down the bot can achieve. LOWER = less overshoot
TURN_MARGIN_DEG = 3.0      # deg, spin has reached ~0 this far before the target
TURN_CREEP = 0.5           # rad/s, slow final creep so the last degrees finish
TURN_YAW_KP = 8.0          # wheel rad/s per rad/s of yaw-rate error
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
_hold_pid = PID(HOLD_KP, HOLD_KI, HOLD_KD, out_limit=STEER_MAX, i_limit=1.0)

# Controller state (module-level so on_message's signature stays untouched)
_state = {
    "mode": "DRIVE",    # DRIVE -> STOP -> TURN -> DRIVE ...
    "heading": 0.0,     # rad, integrated gyro yaw
    "hold": 0.0,        # rad, heading to hold while driving
    "target": 0.0,      # rad, heading goal during a turn
    "turn": 0.0,        # rad, turn chosen when the stop began (+ = left)
    "stop_t": 0.0,      # s, time spent in STOP
    "win_t": 0.0,       # s, time inside the current "is it still?" window
    "win_front": 0.0,   # m, front reading at the start of that window
    "still_n": 0,       # consecutive still windows
    "v_t": 0.0,         # s, time inside the closing-speed window
    "v_front": None,    # m, front reading at the start of that window
    "v_close": 0.0,     # m/s, measured closing speed on the front wall
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


def _update_closing_speed(front, dt):
    """Measure how fast the bot is closing on the front wall (m/s) from the ToF."""
    if _state["v_front"] is None:
        _state["v_front"] = front
    _state["v_t"] += dt
    if _state["v_t"] >= VEL_WIN:
        raw = (_state["v_front"] - front) / _state["v_t"]
        _state["v_close"] = 0.5 * _state["v_close"] + 0.5 * raw
        _state["v_front"], _state["v_t"] = front, 0.0


def _approach_speed(front):
    """Allowed speed at this front distance: slows smoothly so the bot arrives at creep speed."""
    d = max(0.0, front - FRONT_STOP - BRAKE_MARGIN)
    return min(BASE_SPEED, MIN_SPEED + BRAKE_K * math.sqrt(d))


def _turn_command(err, yaw_rate):
    """In-place spin command that follows a yaw-rate profile ending exactly at the target."""
    margin = math.radians(TURN_MARGIN_DEG)
    if abs(err) < TURN_TOL:
        w_des = 0.0
    else:
        speed = math.sqrt(2.0 * TURN_DECEL * max(0.0, abs(err) - margin))
        w_des = math.copysign(min(TURN_YAW_MAX, speed + TURN_CREEP), err)   # + = left
    w = TURN_YAW_KP * (w_des - yaw_rate)
    w = max(-TURN_MAX, min(TURN_MAX, w))
    return _clamp(-w), _clamp(w)


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    """Return (left_vel, right_vel) in rad/s."""
    fl, fr, sl, sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    if SWAP_SIDES:
        sl, sr = sr, sl
    _state["heading"] += yaw_rate * dt          # gyro-integrated heading

    front = min(fl, fr)
    _update_closing_speed(front, dt)
    mode = _state["mode"]

    # ---- STOP: wheels at zero until the bot is really at rest, then start the turn ----
    if mode == "STOP":
        _state["stop_t"] += dt

        if front < BACKOFF_DIST:                # too close to the wall: back off gently
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
            _set_mode("TURN")
        return 0.0, 0.0

    # ---- TURN: rotate in place toward the open side ----
    if mode == "TURN":
        err = _state["target"] - _state["heading"]
        if abs(err) < TURN_TOL and abs(yaw_rate) < 0.3:
            _state["hold"] = _state["target"]   # drive straight along the new heading
            _hold_pid.reset()
            _center_pid.reset()
            _state["v_front"], _state["v_close"] = None, 0.0
            _set_mode("DRIVE")
            return 0.0, 0.0
        return _turn_command(err, yaw_rate)

    # ---- DRIVE: go straight until a wall is detected in front ----
    if front < FRONT_STOP:
        angle = math.radians(TURN_ANGLE_DEG)
        # Turn toward the side whose wall is farther away (the open side).
        _state["turn"] = angle if sl >= sr else -angle
        _state.update(stop_t=0.0, win_t=0.0, win_front=front, still_n=0)
        _set_mode("STOP")
        return 0.0, 0.0

    # Crash guard: closing faster than the bot could shed before FRONT_STOP -> cut / brake.
    v_safe = math.sqrt(2.0 * A_DECEL * max(0.0, front - FRONT_STOP))
    if _state["v_close"] > v_safe:
        if _state["v_close"] > 1.5 * v_safe:
            return -BRAKE_REV, -BRAKE_REV
        return 0.0, 0.0

    speed = _approach_speed(front)

    if sl < OPEN_THRESH and sr < OPEN_THRESH:
        # Walls on both sides: stay centred between them.
        _state["hold"] = _state["heading"]
        u = _center_pid.update(0.5 * (sl - sr), dt)     # + = steer left
        return _clamp(speed - u), _clamp(speed + u)

    # No wall on a side: hold the gyro heading straight.
    _center_pid.reset()
    w = _hold_pid.update(_state["hold"] - _state["heading"], dt)
    return _clamp(speed - w), _clamp(speed + w)


def _log(fl, fr, sl, sr, yaw_rate, dt, left_vel, right_vel):
    """Throttled status line (one per PRINT_EVERY sensor messages)."""
    _state["count"] += 1
    if _state["count"] % PRINT_EVERY == 0:
        front = min(_clean(fl), _clean(fr))
        print(f"{_state['mode']:<6} front={front:.3f} sl={_clean(sl):.3f} sr={_clean(sr):.3f} "
              f"v={_state['v_close']:+.2f}m/s yaw={yaw_rate:+.2f}rad/s "
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

    # Throttled to ~10 Hz (and on every mode change) so printing doesn't slow the loop.
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