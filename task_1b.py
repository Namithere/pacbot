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
FOLLOW_LEFT_WALL = True    # True: track left wall, False: track right wall
BASE_SPEED = 10.0          # rad/s, cruising wheel speed  (was 6.0)
MAX_WHEEL = 16.0           # rad/s, saturation limit      (was 12.0)
MAX_RANGE = 2.0            # m, value used for invalid / infinite ToF readings

WALL_TARGET = 0.10         # m, desired distance to the tracked wall
FRONT_STOP = 0.14          # m, front wall closer than this -> turn (a bit earlier at speed)
OPEN_THRESH = 0.25         # m, side reading above this -> wall opening
ADVANCE_T = 0.15           # s, drive straight into a junction before turning
ENTER_T = 0.24             # s, drive straight after turning to re-find wall

WALL_KP, WALL_KI, WALL_KD = 50.0, 0.5, 5.0     # wall-distance PID
TURN_KP, TURN_KI, TURN_KD = 8.0, 0.0, 0.4      # heading PID (turns)
TURN_MAX = 8.0             # rad/s, max wheel speed during in-place turns
TURN_TOL = math.radians(2.5)   # rad, heading error considered "done"
SLOW_ZONE = 0.20           # m, start easing off the throttle this far past FRONT_STOP


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

# Controller state (module-level so on_message's signature stays untouched)
_state = {
    "mode": "FOLLOW",   # FOLLOW | ADVANCE | TURN | ENTER
    "timer": 0.0,       # s, used by ADVANCE / ENTER
    "heading": 0.0,     # rad, integrated gyro yaw
    "target": 0.0,      # rad, heading goal while TURN
    "next_turn": 0.0,   # rad, turn to apply after ADVANCE (+ = left)
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


def _start_turn(angle):
    """Begin an in-place turn of `angle` rad (+ = left/CCW, - = right/CW)."""
    _state["target"] = _state["heading"] + angle
    _state["mode"] = "TURN"
    _turn_pid.reset()


def _controller(fl, fr, sl, sr, yaw_rate, dt):
    """Return (left_vel, right_vel) in rad/s."""
    fl, fr, sl, sr = _clean(fl), _clean(fr), _clean(sl), _clean(sr)
    _state["heading"] += yaw_rate * dt          # gyro-integrated heading

    front = min(fl, fr)
    side = sl if FOLLOW_LEFT_WALL else sr       # tracked wall distance
    # Turn toward the opening side when a gap appears, away from wall when blocked
    open_turn = math.pi / 2 if FOLLOW_LEFT_WALL else -math.pi / 2
    block_turn = -open_turn

    mode = _state["mode"]

    # ---- TURN: rotate in place using gyro heading ----
    if mode == "TURN":
        err = _state["target"] - _state["heading"]
        if abs(err) < TURN_TOL and abs(yaw_rate) < 0.3:
            _state["mode"] = "ENTER"
            _state["timer"] = ENTER_T
            _wall_pid.reset()
            return 0.0, 0.0
        w = _turn_pid.update(err, dt)           # + = turn left
        return _clamp(-w), _clamp(w)

    # ---- ADVANCE: roll forward into the junction, then turn ----
    if mode == "ADVANCE":
        _state["timer"] -= dt
        if front < FRONT_STOP or _state["timer"] <= 0.0:
            _start_turn(_state["next_turn"])
            return 0.0, 0.0
        return BASE_SPEED, BASE_SPEED

    # ---- ENTER: go straight after a turn so the wall is re-acquired ----
    if mode == "ENTER":
        _state["timer"] -= dt
        if front < FRONT_STOP:
            _start_turn(block_turn)
            return 0.0, 0.0
        if _state["timer"] <= 0.0:
            _state["mode"] = "FOLLOW"
            _wall_pid.reset()
        return BASE_SPEED, BASE_SPEED

    # ---- FOLLOW: PID on distance to the tracked wall ----
    if front < FRONT_STOP:                      # wall ahead (or dead end)
        _start_turn(block_turn)
        return 0.0, 0.0

    if side > OPEN_THRESH:                      # opening on the tracked side
        _state["mode"] = "ADVANCE"
        _state["timer"] = ADVANCE_T
        _state["next_turn"] = open_turn
        return BASE_SPEED, BASE_SPEED

    err = side - WALL_TARGET                    # + = too far from the wall
    u = _wall_pid.update(err, dt)               # + = steer toward the wall
    # Ease off the throttle only when a front wall is close
    speed = BASE_SPEED * min(1.0, max(0.5, (front - FRONT_STOP) / SLOW_ZONE))
    if FOLLOW_LEFT_WALL:
        # steer left (toward wall): right wheel faster
        left, right = speed - u, speed + u
    else:
        left, right = speed + u, speed - u
    return _clamp(left), _clamp(right)
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