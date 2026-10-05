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
# Robot / maze geometry (wheel radius measured from roda_sim.stl: 0.017 m;
# track width estimated from chassis_sim.stl, ~0.092 m - tune if turns
# under/overshoot).
# ----------------------------------------------------------------------------
WHEEL_R = 0.017
TRACK = 0.092
CORRIDOR = 0.22

# ----------------------------------------------------------------------------
# Tunables
# ----------------------------------------------------------------------------
BASE_SPEED = 6.0         # rad/s per wheel while cruising (~0.1 m/s)
MIN_SPEED = 2.0          # rad/s, floor when slowing for a wall ahead
SLOW_DIST = 0.25         # m, start slowing when front reading below this
FRONT_STOP = 0.08        # m, stop & turn when front reading below this
OPEN_DIST = 0.17         # m, a side reading above this = opening / no wall
MAX_VALID = 1.0          # m, readings above this (or non-finite) = no wall

KP = 18.0                # rad/s of wheel differential per metre of error
KI = 4.0
KD = 0.4
I_LIMIT = 3.0
U_LIMIT = 4.0            # max steering differential (rad/s)
D_ALPHA = 0.2            # low-pass factor for derivative term

TURN_RATE_MAX = 2.5      # rad/s body yaw rate during turns
TURN_RATE_MIN = 0.6
TURN_KP = 4.0
TURN_TOL = math.radians(1.5)
PREFER = +1              # +1 = prefer left turns at junctions, -1 = right
                         # (yaw about +z is counter-clockwise = left)

# ----------------------------------------------------------------------------
# Controller state
# ----------------------------------------------------------------------------
state = {
    "mode": "FOLLOW",    # FOLLOW or TURN
    "integ": 0.0,
    "prev_err": None,
    "d_filt": 0.0,
    "angle": 0.0,        # integrated yaw during a turn
    "target": 0.0,       # target yaw change for the current turn
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
    """Positive error -> robot is too close to the right wall -> steer left."""
    half = CORRIDOR / 2.0
    # Treat an "open" side as missing so we don't chase a gap.
    l_ok = _valid(sl) and sl < OPEN_DIST
    r_ok = _valid(sr) and sr < OPEN_DIST
    if l_ok and r_ok:
        return sl - sr
    if l_ok:
        return 2.0 * (sl - (half - 0.0))  # hold ~half-corridor from left wall
    if r_ok:
        return 2.0 * ((half - 0.0) - sr)
    return None


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
        turn = math.pi          # dead end
    state.update(mode="TURN", angle=0.0, target=turn)
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

    fl_v = fl if _valid(fl) else MAX_VALID
    fr_v = fr if _valid(fr) else MAX_VALID
    front = min(fl_v, fr_v)

    left_vel = 0.0
    right_vel = 0.0

    if state["mode"] == "TURN":
        # Gyro-integrated angle tells us how far we have really rotated.
        state["angle"] += yaw_rate * dt
        err = state["target"] - state["angle"]
        if abs(err) < TURN_TOL:
            state["mode"] = "FOLLOW"
            _reset_pid()
        else:
            w = _clamp(TURN_KP * abs(err), TURN_RATE_MIN, TURN_RATE_MAX)
            w = math.copysign(w, err)
            wheel = w * (TRACK / 2.0) / WHEEL_R   # in-place rotation
            left_vel, right_vel = -wheel, wheel
    else:
        if front < FRONT_STOP:
            _start_turn(sl, sr)
        else:
            # Slow down as a wall approaches.
            k = _clamp((front - FRONT_STOP) / (SLOW_DIST - FRONT_STOP), 0.0, 1.0)
            speed = MIN_SPEED + k * (BASE_SPEED - MIN_SPEED)

            err = _centering_error(sl, sr)
            if err is None:
                u = 0.0
                _reset_pid()
            else:
                state["integ"] = _clamp(state["integ"] + err * dt,
                                        -I_LIMIT / max(KI, 1e-9) if KI else 0,
                                        I_LIMIT / max(KI, 1e-9) if KI else 0)
                if state["prev_err"] is None:
                    d_raw = 0.0
                else:
                    d_raw = (err - state["prev_err"]) / dt
                state["d_filt"] += D_ALPHA * (d_raw - state["d_filt"])
                state["prev_err"] = err
                u = KP * err + KI * state["integ"] + KD * state["d_filt"]
                u = _clamp(u, -U_LIMIT, U_LIMIT)

            # u > 0 steers left: left wheel slower, right wheel faster.
            left_vel = speed - u
            right_vel = speed + u

    print(f"{state['mode']:6s} fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f} "
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
        pass