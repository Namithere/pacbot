"""PB Task 1B - straight driving, stop at front wall, 90 degree turn toward
the side with more space.

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
# Geometry (wheel radius from roda_sim.stl; track width estimated from
# chassis_sim.stl).
# ----------------------------------------------------------------------------
WHEEL_R = 0.017
TRACK = 0.092

# ----------------------------------------------------------------------------
# Tunables
# ----------------------------------------------------------------------------
BASE_SPEED = 6.0          # rad/s per wheel while cruising
MIN_SPEED = 1.5           # rad/s, creep speed right before the stop point
START_STRAIGHT_TIME = 1.0 # s of straight driving with front sensors ignored

# Front wall detection / collision avoidance.
# "front" = max(fl, fr): both front sensors must see the wall.
FRONT_STOP = 0.15         # m, stop and turn below this
FRONT_EMERGENCY = 0.11    # m, brake immediately (no confirmation) below this
SLOW_DIST = 0.35          # m, begin slowing below this
FRONT_CONFIRM = 3         # consecutive samples below FRONT_STOP to trigger
MAX_VALID = 1.0           # m, readings above this / non-finite = open space

# Gyro heading hold
K_HEAD = 6.0              # rad/s differential per rad of heading error
K_GYRO = 1.0              # damping on yaw rate
U_LIMIT = 3.0             # max steering differential (rad/s)

# Side collision guard (only acts when very close to a side wall;
# set SIDE_SAFE = 0 to disable)
SIDE_SAFE = 0.06          # m
K_SIDE = 40.0             # rad/s differential per metre inside SIDE_SAFE

# Stop-and-turn
BRAKE_TIME = 0.35         # s at zero speed to let the robot stop
BIAS_TIME = 0.15          # s stationary: gyro bias + side readings averaged
TURN_RATE_MAX = 2.0       # rad/s body yaw rate
TURN_RATE_MIN = 0.5
TURN_KP = 4.0
TURN_TOL = math.radians(1.0)
SETTLE_TIME = 0.3         # s stationary after turn, gyro still integrating
SETTLE_TOL = math.radians(2.0)   # re-turn if still off by more than this
TURN_ANGLE = math.pi / 2  # +left (CCW about +z), -right

# ----------------------------------------------------------------------------
# State
# ----------------------------------------------------------------------------
state = {
    "mode": "START",      # START -> DRIVE -> BRAKE -> TURN -> SETTLE -> DRIVE
    "t": 0.0,
    "t_mode": 0.0,
    "front_count": 0,
    "heading": 0.0,       # integrated yaw while driving (rad)
    "angle": 0.0,         # integrated yaw during a turn
    "target": 0.0,
    "bias": 0.0,          # gyro z bias, measured while stopped
    "bias_sum": 0.0,
    "sl_sum": 0.0,
    "sr_sum": 0.0,
    "n": 0,
    "seen": {k: [1e9, -1e9] for k in ("fl", "fr", "sl", "sr")},
}


def _valid(x):
    return x is not None and math.isfinite(x) and 0.0 < x < MAX_VALID


def _val(x):
    return x if _valid(x) else MAX_VALID


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def _start_brake():
    state.update(mode="BRAKE", t_mode=0.0, angle=0.0, front_count=0,
                 bias_sum=0.0, sl_sum=0.0, sr_sum=0.0, n=0)


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
    rate = yaw_rate - state["bias"]

    for k, v in (("fl", fl), ("fr", fr), ("sl", sl), ("sr", sr)):
        state["seen"][k][0] = min(state["seen"][k][0], v)
        state["seen"][k][1] = max(state["seen"][k][1], v)

    # Both front sensors must see a close wall for it to count.
    front = max(_val(fl), _val(fr))
    mode = state["mode"]
    left_vel = 0.0
    right_vel = 0.0

    if mode in ("START", "DRIVE"):
        state["heading"] += rate * dt

        speed = BASE_SPEED
        if mode == "START":
            if state["t"] >= START_STRAIGHT_TIME:
                state["mode"] = "DRIVE"
        else:
            state["front_count"] = state["front_count"] + 1 \
                if front < FRONT_STOP else 0
            if state["front_count"] >= FRONT_CONFIRM or front < FRONT_EMERGENCY:
                _start_brake()          # wheels stay at 0 from this step on
            else:
                k = _clamp((front - FRONT_STOP) / (SLOW_DIST - FRONT_STOP),
                           0.0, 1.0)
                speed = MIN_SPEED + k * (BASE_SPEED - MIN_SPEED)

        if state["mode"] in ("START", "DRIVE"):
            # Hold heading with the gyro.
            u = -K_HEAD * state["heading"] - K_GYRO * rate

            # Side collision guard: push away from a very close wall and
            # accept the new direction as "straight".
            push = 0.0
            if SIDE_SAFE > 0:
                if _valid(sl) and sl < SIDE_SAFE:
                    push -= K_SIDE * (SIDE_SAFE - sl)
                if _valid(sr) and sr < SIDE_SAFE:
                    push += K_SIDE * (SIDE_SAFE - sr)
            if push != 0.0:
                state["heading"] = 0.0
            u = _clamp(u + push, -U_LIMIT, U_LIMIT)

            left_vel = speed - u    # u > 0 steers left
            right_vel = speed + u

    elif mode == "BRAKE":
        # Wheels at zero. After the robot has stopped, average the gyro bias
        # and side readings, then choose the more open side.
        state["t_mode"] += dt
        if state["t_mode"] >= BRAKE_TIME:
            state["bias_sum"] += yaw_rate
            state["sl_sum"] += _val(sl)
            state["sr_sum"] += _val(sr)
            state["n"] += 1
            if state["t_mode"] >= BRAKE_TIME + BIAS_TIME:
                n = max(state["n"], 1)
                state["bias"] = state["bias_sum"] / n
                left_space = state["sl_sum"] / n
                right_space = state["sr_sum"] / n
                state["target"] = TURN_ANGLE if left_space >= right_space \
                    else -TURN_ANGLE
                state["angle"] = 0.0
                state["mode"] = "TURN"

    elif mode == "TURN":
        state["angle"] += rate * dt
        err = state["target"] - state["angle"]
        if abs(err) < TURN_TOL:
            state.update(mode="SETTLE", t_mode=0.0)
        else:
            w = math.copysign(
                _clamp(TURN_KP * abs(err), TURN_RATE_MIN, TURN_RATE_MAX), err)
            wheel = w * (TRACK / 2.0) / WHEEL_R   # in-place rotation
            left_vel, right_vel = -wheel, wheel

    elif mode == "SETTLE":
        state["angle"] += rate * dt
        state["t_mode"] += dt
        if state["t_mode"] >= SETTLE_TIME:
            if abs(state["target"] - state["angle"]) > SETTLE_TOL:
                state["mode"] = "TURN"            # correct residual error
            else:
                state.update(mode="DRIVE", heading=0.0, front_count=0)

    print(f"{state['mode']:6s} front={front:.3f} "
          f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"ang={math.degrees(state['angle']):+.1f} "
          f"L={left_vel:+.2f} R={right_vel:+.2f}")

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