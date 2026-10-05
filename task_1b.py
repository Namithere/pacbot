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
SPEED_SCALE = 10.0         # overall speed multiplier (1.0 = original speed)
BASE_SPEED = 10.0 * SPEED_SCALE    # rad/s, cruising wheel speed
MAX_WHEEL = 16.0 * SPEED_SCALE     # rad/s, saturation limit
MIN_SPEED = 8.0            # rad/s, creep speed when right at a wall
MAX_RANGE = 2.0            # m, value used for invalid / infinite ToF readings

WALL_TARGET = 0.05         # m, desired distance to a side wall while driving
FRONT_STOP = 0.09          # m, front wall closer than this -> stop and turn
BRAKE_ZONE = 0.60          # m, start slowing this far before FRONT_STOP
OPEN_THRESH = 0.15         # m, a side wall farther than this is ignored (gap)

WALL_KP, WALL_KI, WALL_KD = 80.0, 0.5, 6.0     # wall-distance PID
TURN_KP, TURN_KI, TURN_KD = 8.0, 0.0, 0.4      # heading PID (turns)
HOLD_KP, HOLD_KI, HOLD_KD = 8.0, 0.0, 0.2      # heading PID (straight-line hold)
TURN_MAX = 8.0             # rad/s, max wheel speed during in-place turns
HOLD_MAX = 4.0 * SPEED_SCALE   # rad/s, max correction while holding heading
TURN_TOL = math.radians(2.5)   # rad, heading error considered "done"


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


_wall_pid = PID(WALL_KP, WALL_KI, WALL_KD, out_limit=BASE_SPEED, i_limit=2.0)
_turn_pid = PID(TURN_KP, TURN_KI, TURN_KD, out_limit=TURN_MAX, i_limit=1.0)
_hold_pid = PID(HOLD_KP, HOLD_KI, HOLD_KD, out_limit=HOLD_MAX, i_limit=1.0)

# Controller state (module-level so on_message's signature stays untouched)
_state = {
    "mode": "STRAIGHT", # STRAIGHT <-> TURN
    "heading": 0.0,     # rad, integrated gyro yaw
    "hold": 0.0,        # rad, heading to hold on straight runs
    "target": 0.0,      # rad, heading goal while TURN
    "wall": False,      # True while a side wall is being tracked
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


def _approach_speed(front):
    """Cruise speed scaled down linearly as a front wall gets closer.

    Full speed at FRONT_STOP + BRAKE_ZONE or farther, MIN_SPEED at FRONT_STOP.
    """
    frac = (front - FRONT_STOP) / BRAKE_ZONE
    frac = max(0.0, min(1.0, frac))
    return MIN_SPEED + (BASE_SPEED - MIN_SPEED) * frac


def _start_turn(angle):
    """Begin an in-place turn of `angle` rad (+ = left/CCW, - = right/CW)."""
    _state["target"] = _state["heading"] + angle
    _state["mode"] = "TURN"
    _turn_pid.reset()


def _hold_straight(speed, dt):
    """Drive at `speed` while holding the stored heading using the gyro."""
    err = _state["hold"] - _state["heading"]
    w = _hold_pid.update(err, dt)               # + = steer left
    return _clamp(speed - w), _clamp(speed + w)


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    """Return (left_vel, right_vel) in rad/s."""
    fl, fr, sl, sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    _state["heading"] += yaw_rate * dt          # gyro-integrated heading

    front = min(fl, fr)
    speed = _approach_speed(front)              # brakes automatically near walls

    # ---- TURN: rotate in place using gyro heading ----
    if _state["mode"] == "TURN":
        err = _state["target"] - _state["heading"]
        if abs(err) < TURN_TOL and abs(yaw_rate) < 0.3:
            _state["hold"] = _state["target"]   # new straight-line reference
            _hold_pid.reset()
            _wall_pid.reset()
            _state["wall"] = False
            _state["mode"] = "STRAIGHT"         # resume driving forward
            return 0.0, 0.0
        w = _turn_pid.update(err, dt)           # + = turn left
        return _clamp(-w), _clamp(w)

    # ---- STRAIGHT: drive until a wall is detected in front, then turn to the open side ----
    if front < FRONT_STOP:
        _start_turn(math.pi / 2 if sl >= sr else -math.pi / 2)
        return 0.0, 0.0

    # Keep a steady distance from a side wall while one is present.
    # A side opening (gap) is ignored: the bot just holds its heading past it.
    side = min(sl, sr)
    if side < OPEN_THRESH:
        wall_left = sl <= sr
        if not _state["wall"]:
            _wall_pid.reset()
            _state["wall"] = True
        _state["hold"] = _state["heading"]      # keep hold-heading current for smooth hand-over
        err = side - WALL_TARGET                # + = too far from the wall
        u = _wall_pid.update(err, dt)           # + = steer toward the wall
        if wall_left:
            left, right = speed - u, speed + u  # steer left toward the wall
        else:
            left, right = speed + u, speed - u  # steer right toward the wall
        return _clamp(left), _clamp(right)

    _state["wall"] = False
    return _hold_straight(speed, dt)            # no side wall: go straight on gyro heading
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

    # (per-step print removed: printing at ~500 Hz stalls the control loop)

    # Compute wheel velocities (rad/s) from the readings above.
    left_vel, right_vel = _controller(fl, fr, sl, sr, yaw_rate, dt)

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