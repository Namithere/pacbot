"""PB Task 1B - corridor following with PID + gyro-integrated turns.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b.py
"""
import json
import math

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

# ----------------------------------------------------------------------------
# Robot / maze geometry (wheel radius from roda_sim.stl = 0.017 m; track width
# estimated from chassis_sim.stl, ~0.092 m - tune if turns under/overshoot).
# ----------------------------------------------------------------------------
WHEEL_R = 0.017
TRACK = 0.092
CORRIDOR = 0.22

# ----------------------------------------------------------------------------
# Tunables
# ----------------------------------------------------------------------------
BASE_SPEED = 6.0          # rad/s per wheel while cruising (~0.1 m/s)
MIN_SPEED = 2.0           # rad/s, floor when slowing for a wall ahead

# Startup: drive straight, front sensors ignored.
START_STRAIGHT_TIME = 1.0  # s of straight driving before front detection

# Front-wall detection
FRONT_ARM_DIST = 0.20     # front must read above this before the trigger arms
FRONT_STOP = 0.12         # m, start a turn when front reads below this
SLOW_DIST = 0.30          # m, start slowing when front reading below this
FRONT_CONFIRM = 5         # consecutive samples below FRONT_STOP to trigger

OPEN_DIST = 0.17          # m, a side reading above this = opening / no wall
MAX_VALID = 1.0           # m, readings above this (or non-finite) = no wall

# Wall-centering PID
KP = 18.0                 # rad/s of wheel differential per metre of error
KI = 4.0
KD = 0.4
I_LIMIT = 3.0             # limit on the integral *contribution* (rad/s)
U_LIMIT = 4.0             # max steering differential (rad/s)
D_ALPHA = 0.2             # low-pass factor for derivative term

# Gyro heading hold (used at start and when no side wall is visible)
K_HEAD = 6.0              # rad/s of differential per rad of heading error
K_GYRO = 1.0              # damping on yaw rate

# Turns
TURN_RATE_MAX = 2.5       # rad/s body yaw rate during turns
TURN_RATE_MIN = 0.6
TURN_KP = 4.0
TURN_TOL = math.radians(1.5)
PREFER = +1               # +1 = prefer left at junctions, -1 = right
                          # (yaw about +z is counter-clockwise = left)

# ----------------------------------------------------------------------------
# Controller state
# ----------------------------------------------------------------------------
state = {
    "mode": "START",      # START -> FOLLOW <-> TURN
    "t": 0.0,             # time since launch (s)
    "armed": False,       # front-wall trigger armed?
    "front_count": 0,     # consecutive samples below FRONT_STOP
    "heading": 0.0,       # integrated yaw since last reset (rad)
    "integ": 0.0,
    "prev_err": None,
    "d_filt": 0.0,
    "angle": 0.0,         # integrated yaw during a turn
    "target": 0.0,        # target yaw change for the current turn
    "seen": {k: [1e9, -1e9] for k in ("fl", "fr", "sl", "sr")},
}


def _valid(x):
    return x is not None and math.isfinite(x) and 0.0 < x < MAX_VALID


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _reset_pid():
    state["integ"] = 0.0
    state["prev_err"] = None
    state["d_filt"] = 0.0


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def _centering_error(sl, sr):
    """Positive error -> robot too close to the right wall -> steer left.
    Returns None if no side wall is visible."""
    half = CORRIDOR / 2.0
    l_ok = _valid(sl) and sl < OPEN_DIST
    r_ok = _valid(sr) and sr < OPEN_DIST
    if l_ok and r_ok:
        return sl - sr
    if l_ok:
        return 2.0 * (sl - half)   # hold half a corridor from the left wall
    if r_ok:
        return 2.0 * (half - sr)   # hold half a corridor from the right wall
    return None


def _heading_hold(yaw_rate):
    """Steering output that keeps the robot pointing along heading = 0."""
    u = -K_HEAD * state["heading"] - K_GYRO * yaw_rate
    return _clamp(u, -U_LIMIT, U_LIMIT)


def _start_turn(sl, sr):
    l_open = (not _valid(sl)) or sl > OPEN_DIST
    r_open = (not _valid(sr)) or sr > OPEN_DIST
    if l_open and r_open:
        turn = PREFER * math.pi / 2
    elif l_open:
        turn = math.pi / 2
    elif r_open:
        turn = -math.pi / 2
    else:
        turn = math.pi           # dead end
    state.update(mode="TURN", angle=0.0, target=turn, front_count=0,
                 armed=False)
    _reset_pid()


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings
    sr = data["sr"]            # Side-right ToF distance readings
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    if dt is None or dt <= 0:
        dt = 0.002

    state["t"] += dt
    state["heading"] += yaw_rate * dt

    for k, v in (("fl", fl), ("fr", fr), ("sl", sl), ("sr", sr)):
        state["seen"][k][0] = min(state["seen"][k][0], v)
        state["seen"][k][1] = max(state["seen"][k][1], v)

    fl_v = fl if _valid(fl) else MAX_VALID
    fr_v = fr if _valid(fr) else MAX_VALID
    front = min(fl_v, fr_v)

    left_vel = 0.0
    right_vel = 0.0

    if state["mode"] == "START":
        # Straight line, gyro-held, front sensors ignored.
        u = _heading_hold(yaw_rate)
        left_vel = BASE_SPEED - u
        right_vel = BASE_SPEED + u
        if state["t"] >= START_STRAIGHT_TIME:
            state["mode"] = "FOLLOW"
            state["heading"] = 0.0
            _reset_pid()

    elif state["mode"] == "TURN":
        # Gyro-integrated angle tells us how far we have really rotated.
        state["angle"] += yaw_rate * dt
        err = state["target"] - state["angle"]
        if abs(err) < TURN_TOL:
            state["mode"] = "FOLLOW"
            state["heading"] = 0.0
            _reset_pid()
        else:
            w = _clamp(TURN_KP * abs(err), TURN_RATE_MIN, TURN_RATE_MAX)
            w = math.copysign(w, err)
            wheel = w * (TRACK / 2.0) / WHEEL_R   # in-place rotation
            left_vel, right_vel = -wheel, wheel

    else:  # FOLLOW
        # Arm the front trigger only after the sensors have shown open space.
        if not state["armed"] and front > FRONT_ARM_DIST:
            state["armed"] = True
            state["front_count"] = 0

        if state["armed"]:
            state["front_count"] = state["front_count"] + 1 \
                if front < FRONT_STOP else 0
            if state["front_count"] >= FRONT_CONFIRM:
                _start_turn(sl, sr)

        if state["mode"] == "FOLLOW":      # no turn was just started
            if state["armed"]:
                k = _clamp((front - FRONT_STOP) / (SLOW_DIST - FRONT_STOP),
                           0.0, 1.0)
                speed = MIN_SPEED + k * (BASE_SPEED - MIN_SPEED)
            else:
                speed = BASE_SPEED

            err = _centering_error(sl, sr)
            if err is None:
                # No side walls: hold heading with the gyro.
                u = _heading_hold(yaw_rate)
                _reset_pid()
            else:
                state["heading"] = 0.0     # wall PID owns steering now
                i_max = I_LIMIT / KI if KI else 0.0
                state["integ"] = _clamp(state["integ"] + err * dt,
                                        -i_max, i_max)
                d_raw = 0.0 if state["prev_err"] is None \
                    else (err - state["prev_err"]) / dt
                state["d_filt"] += D_ALPHA * (d_raw - state["d_filt"])
                state["prev_err"] = err
                u = KP * err + KI * state["integ"] + KD * state["d_filt"]
                u = _clamp(u, -U_LIMIT, U_LIMIT)

            # u > 0 steers left: left wheel slower, right wheel faster.
            left_vel = speed - u
            right_vel = speed + u

    print(f"{state['mode']:6s} arm={int(state['armed'])} "
          f"fl={fl} fr={fr} sl={sl} sr={sr} "
          f"yaw_rate={yaw_rate:+.3f} L={left_vel:+.2f} R={right_vel:+.2f}")

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
        print("\nsensor min/max seen:", state["seen"])