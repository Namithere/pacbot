"""PacBot Task 2A - solver boilerplate.

Fill in the parts marked TODO. Everything else - the MQTT connection, the
message formats, the maze layout and the robot's dimensions - is done for
you and matches the simulator.

The task: the robot spawns in a random cell facing a random open direction.
Drive through the pellet's cell, then leave the maze through the exit on
the -y (south) side of cell (14, 14). Score: 20 for solving + 10 for the pellet
- 5 per wall collision.

What you get over MQTT:
  pacbot/poses    once at startup (retained): the robot's spawn pose
                  -> Solver.on_poses()
  pacbot/pellets  once at startup (retained): the pellet's pose
                  -> Solver.on_pellets()
  pacbot/sensors  every physics step (10 ms): ToF distances, gyro,
                  accelerometer, wheel encoder ticks -> Solver.step()
  pacbot/result   once, when the run ends: {"solved", "collisions"}
What you send:
  pacbot/wheel_vel  wheel speeds in rad/s - the return value of Solver.step()

Run:
    ./task_2a_launch                 # terminal 1
    python3 task_2a.py               # terminal 2
"""
import heapq
import json
import math

import paho.mqtt.client as mqtt

# ------------------------------------------------------------------ MQTT
MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # sim -> us, every physics step
TOPIC_POSES = "pacbot/poses"          # sim -> us, once, retained: robot spawn pose
TOPIC_PELLETS = "pacbot/pellets"      # sim -> us, once, retained: pellet pose
TOPIC_RESULT = "pacbot/result"        # sim -> us, once, when the run ends
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # us -> sim

# ------------------------------------------------------------------ maze
N = 15            # cells per side
PITCH = 0.22      # m, cell centre to cell centre
WALL_HALF = 0.005  # m, half the wall thickness
EXIT = (N - 1, N - 1, 3)  # the only opening in the border: -y side of (14, 14)

# Directions: 0 = +x, 1 = +y, 2 = -x, 3 = -y
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

DIR_TO_ANGLE = {
    0: 0.0,
    1: math.pi / 2,
    2: math.pi,
    3: -math.pi / 2,
}

LAYOUT = (
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+",
    "|                           |       |               |       |",
    "+---+---+   +---+   +   +   +   +   +---+   +---+   +---+   +",
    "|           |           |   |   |       |       |           |",
    "+   +   +   +   +---+---+---+   +---+   +   +   +---+---+   +",
    "|   |           |               |   |   |   |   |   |       |",
    "+   +---+---+---+   +---+   +---+   +   +---+   +   +   +---+",
    "|   |               |               |       |   |           |",
    "+   +   +---+---+---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |           |       |   |       |       |       |",
    "+   +---+   +   +   +---+---+   +   +   +---+---+   +---+   +",
    "|           |   |       |       |   |       |       |   |   |",
    "+   +   +   +   +---+   +   +---+   +---+   +   +---+   +   +",
    "|   |       |       |   |   |   |       |       |       |   |",
    "+   +   +   +---+---+   +   +   +   +   +   +---+   +   +   +",
    "|   |   |           |       |       |           |   |       |",
    "+   +   +---+---+   +   +---+---+   +---+---+   +   +---+---+",
    "|   |   |           |           |       |       |           |",
    "+   +   +   +---+---+---+---+   +   +   +   +---+---+---+   +",
    "|   |   |               |       |   |   |           |       |",
    "+   +---+---+   +---+   +   +---+   +   +---+   +   +   +---+",
    "|   |       |   |           |           |       |   |   |   |",
    "+   +   +   +   +   +---+   +---+---+   +   +---+   +   +   +",
    "|   |   |           |   |               |   |               |",
    "+   +   +---+---+---+   +---+---+---+   +   +---+   +---+   +",
    "|   |       |                   |       |       |       |   |",
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +   +",
    "|   |   |       |   |           |   |       |   |       |   |",
    "+   +   +---+---+   +   +   +   +   +---+   +   +---+---+   +",
    "|               |           |               |               |",
    "+---+---+---+---+---+---+---+---+---+---+---+---+---+---+ E +",
)


def _wall_table():
    walls = {}
    for row in range(N):
        for col in range(N):
            r, c = 2 * row + 1, 4 * col
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"                # +x: right
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"    # +y: above
            walls[row, col, 2] = LAYOUT[r][c] == "|"                    # -x: left
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"    # -y: below
    return walls


WALLS = _wall_table()


def cell_centre(cell):
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           # m, wheel radius
TRACK = 0.078             # m, wheel centre to wheel centre
TICKS_PER_REV = 360       # encoder ticks per wheel revolution
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0          # rad/s, simulator clamp
AXLE_BACK = 0.033         # m, wheel axle distance behind chassis centre

SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wheels(v, omega):
    left = (v - omega * TRACK / 2.0) / WHEEL_R
    right = (v + omega * TRACK / 2.0) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)


# ------------------------------------------------------- controller tuning
V_CRUISE = 0.32        # m/s, forward cruising speed
V_MIN = 0.04           # m/s, creep speed near stop point
A_ACC = 1.2            # m/s^2, acceleration ramp
A_DEC = 1.0            # m/s^2, braking profile
A_DEC_MAX = 1.8        # m/s^2, max commanded deceleration
STOP_TOL = 0.005       # m, stop distance tolerance

KH = 6.0               # heading P gain on straight run
KL = 5.0               # lateral P gain on straight run
MAX_REF = 0.30         # rad, lane heading offset clamp

KP_TURN = 7.0          # turn P gain
KD_TURN = 0.45         # turn D (gyro damping) gain to stop overshoots
W_TURN_MAX = 3.2       # rad/s, max turn speed
W_TURN_MIN = 0.35      # rad/s, min turn speed to prevent stalling
TURN_TOL = math.radians(2.0)   # 2.0 degrees target tolerance
GYRO_TOL = 0.20        # rad/s, stabilized threshold

HALF_GAP = (PITCH - 2 * WALL_HALF) / 2
WALL_NEAR = 0.19
SIDE_OFF = 0.005
SENSOR_ARM = AXLE_BACK + 0.036
FRONT_ARM = AXLE_BACK - 0.005
FRONT_RANGE = 0.8
LAT_GAIN = 0.10
FWD_GAIN = 0.30


# ============================================================== planning

def plan_route(start, heading, pellet):
    start, pellet = tuple(start), tuple(pellet)
    turn_cost = {1: 2.0, 3: 2.0, 2: 3.0}
    first = (start[0], start[1], heading, start == pellet)
    best = {first: 0.0}
    back = {}
    tie = 0
    heap = [(0.0, tie, first)]
    goal = None
    while heap:
        cost, _, state = heapq.heappop(heap)
        if cost > best.get(state, float("inf")):
            continue
        row, col, d, got = state
        if row == N:
            if got:
                goal = state
                break
            continue
        options = []
        for nd in range(4):
            if nd != d:
                options.append(((row, col, nd, got), turn_cost[(nd - d) % 4],
                                ("turn", nd)))
        if not WALLS[row, col, d]:
            nxt = (row + STEP[d][0], col + STEP[d][1])
            if inside(*nxt) or (row, col, d) == EXIT:
                options.append(((nxt[0], nxt[1], d, got or nxt == pellet), 1.0,
                                ("move", d)))
        for new, c, act in options:
            nc = cost + c
            if nc < best.get(new, float("inf")):
                best[new] = nc
                back[new] = (state, act)
                tie += 1
                heapq.heappush(heap, (nc, tie, new))
    if goal is None:
        return []
    raw = []
    state = goal
    while state != first:
        state, act = back[state]
        raw.append(act)
    raw.reverse()
    actions = []
    for kind, d in raw:
        if kind == "move":
            if actions and actions[-1][0] == "run":
                actions[-1] = ("run", actions[-1][1] + 1)
            else:
                actions.append(("run", 1))
        else:
            actions.append(("turn", d))
    return actions


# ================================================================ solver

class Solver:
    def __init__(self):
        self.t = 0.0
        self.spawn = None
        self.pellet = None
        self.actions = None
        self.prev_ticks = None
        self.done = False
        self.x = self.y = self.th = 0.0
        self.heading = 0
        self.cell = None
        self.queue = []
        self.phase = "idle"
        self.settle = 0
        self.v_cmd = 0.0
        self.run_c = (0.0, 0.0)
        self.run_th = 0.0
        self.run_cell = None
        self.run_front_wall = False
        self.turn_target = 0.0
        self.turn_dir = 0
        self.ok_count = 0
        self.gyro_bias = 0.0

    def log(self, text):
        print(f"[{self.t:7.2f}s] {text}", flush=True)

    def on_poses(self, msg):
        bot, d = msg.get("bot"), msg.get("dir")
        if isinstance(bot, list) and d is not None and self.actions is None:
            self.spawn = (d, tuple(bot))
            self.log(f"spawn {self.spawn[1]} facing {self.spawn[0]}")

    def on_pellets(self, msg):
        pellet = msg.get("pellet")
        if pellet and self.actions is None:
            self.pellet = tuple(pellet)
            self.log(f"pellet {self.pellet}")

    def step(self, s):
        self.t += s["dt"]
        ticks = (s["ticks_l"], s["ticks_r"])
        if self.prev_ticks is None:
            self.prev_ticks = ticks
        d_left = (ticks[0] - self.prev_ticks[0]) * M_PER_TICK
        d_right = (ticks[1] - self.prev_ticks[1]) * M_PER_TICK
        self.prev_ticks = ticks
        turn_rate = s["gyro"][2]

        if self.done or self.spawn is None or self.pellet is None:
            return 0.0, 0.0
        if self.actions is None:
            self.actions = plan_route(self.spawn[1], self.spawn[0], self.pellet)
            self.log(f"plan: {self.actions}")
            self._start()

        dt = s["dt"]
        if self.phase == "settle" and d_left == 0 and d_right == 0:
            self.gyro_bias += 0.1 * (turn_rate - self.gyro_bias)

        # Gyro odometry with calibrated bias removal
        omega = turn_rate - self.gyro_bias
        dth = omega * dt
        mid = self.th + 0.5 * dth
        ds = 0.5 * (d_left + d_right)
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th = wrap(self.th + dth)

        return self._control(s, omega)

    def _start(self):
        d, cell = self.spawn
        self.heading = d
        self.cell = tuple(cell)
        self.th = DIR_TO_ANGLE[d]
        cx, cy = cell_centre(cell)
        self.x = cx - AXLE_BACK * math.cos(self.th)
        self.y = cy - AXLE_BACK * math.sin(self.th)
        self.queue = [("run", 0)] + list(self.actions)
        self.phase = "settle"
        self.settle = 40
        self.v_cmd = 0.0

    def _control(self, s, omega):
        while self.phase == "idle":
            if not self.queue:
                self.phase = "finished"
                break
            self._begin(self.queue.pop(0))
        if self.phase == "settle":
            self.settle -= 1
            if self.settle <= 0:
                self.phase = "idle"
            return 0.0, 0.0
        if self.phase == "run":
            return self._run(s)
        if self.phase == "turn":
            return self._turn(omega)
        return 0.0, 0.0

    def _begin(self, action):
        kind, val = action
        if kind == "turn":
            if val == self.heading:
                return
            self.turn_dir = val
            self.turn_target = DIR_TO_ANGLE[val]
            self.ok_count = 0
            self.phase = "turn"
            self.log(f"turn to {val} (target_th: {self.turn_target:.2f})")
        else:
            d = self.heading
            tgt = (self.cell[0] + val * STEP[d][0], self.cell[1] + val * STEP[d][1])
            self.run_cell = tgt
            self.run_c = cell_centre(tgt)
            self.run_th = DIR_TO_ANGLE[d]
            self.run_front_wall = inside(*tgt) and WALLS[tgt[0], tgt[1], d]
            self.v_cmd = 0.0
            self.phase = "run"
            self.log(f"run {val} -> cell {tgt}")

    def _turn(self, omega):
        err = wrap(self.turn_target - self.th)

        # Stable check: within tolerance and essentially stationary
        if abs(err) < TURN_TOL:
            if abs(omega) < GYRO_TOL:
                self.ok_count += 1
            else:
                self.ok_count = 0
        else:
            self.ok_count = 0

        # Lock turn immediately after staying on heading for 2 frames
        if self.ok_count >= 2:
            self.heading = self.turn_dir
            self.th = self.turn_target  # Pin exact heading to kill odometry drift
            self.phase = "settle"
            self.settle = 3             # 30 ms brief stabilization
            return 0.0, 0.0

        # PD turn control: P drives toward angle, D (gyro damping) actively brakes
        w = KP_TURN * err - KD_TURN * omega
        w = clamp(w, -W_TURN_MAX, W_TURN_MAX)

        # Enforce minimum turning speed ONLY outside 1.7 degrees to eliminate hunting
        if abs(err) > 0.03 and abs(w) < W_TURN_MIN:
            w = math.copysign(W_TURN_MIN, err)

        return wheels(0.0, w)

    def _correct(self, s, ux, uy, nx, ny, eps):
        cx, cy = self.run_c
        fl, fr = s["fl"], s["fr"]
        e = None
        if fl < WALL_NEAR and fr < WALL_NEAR:
            e = 0.5 * (fr - fl)
        elif fl < WALL_NEAR:
            e = (HALF_GAP - SIDE_OFF) - fl
        elif fr < WALL_NEAR:
            e = fr - (HALF_GAP - SIDE_OFF)

        if e is not None:
            e -= SENSOR_ARM * math.sin(eps)
            cur = (self.x - cx) * nx + (self.y - cy) * ny
            k = LAT_GAIN * (e - cur)
            self.x += k * nx
            self.y += k * ny

        if self.run_front_wall:
            r = min(s["sl"], s["sr"]) * math.cos(eps)
            if r < FRONT_RANGE:
                meas = (HALF_GAP - FRONT_ARM) - r
                cur = (self.x - cx) * ux + (self.y - cy) * uy
                k = FWD_GAIN * (meas - cur)
                self.x += k * ux
                self.y += k * uy

    def _run(self, s):
        dt = s["dt"]
        ux, uy = math.cos(self.run_th), math.sin(self.run_th)
        nx, ny = -uy, ux                      # Normal vector to corridor
        cx, cy = self.run_c
        eps = wrap(self.th - self.run_th)

        if abs(eps) < 0.15:
            self._correct(s, ux, uy, nx, ny, eps)

        along = (self.x - cx) * ux + (self.y - cy) * uy
        lateral = (self.x - cx) * nx + (self.y - cy) * ny
        rem = -along

        # Stop and snap to cell centre
        if rem <= STOP_TOL:
            self.x -= along * ux
            self.y -= along * uy
            self.cell = self.run_cell
            if abs(eps) < 0.12:
                self.th = self.run_th
            self.v_cmd = 0.0
            self.phase = "settle"
            self.settle = 3
            return 0.0, 0.0

        # Smooth trapezoidal/deceleration profile
        v_des = clamp(math.sqrt(2 * A_DEC * rem), V_MIN, V_CRUISE)
        if v_des > self.v_cmd:
            self.v_cmd = min(v_des, self.v_cmd + A_ACC * dt)
        else:
            self.v_cmd = max(v_des, self.v_cmd - A_DEC_MAX * dt)

        # Steer strictly down the lane centerline
        ref = self.run_th - clamp(KL * lateral, -MAX_REF, MAX_REF)
        omega = KH * wrap(ref - self.th)
        return wheels(self.v_cmd, omega)


# ================================================================== main

def _client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def main():
    solver = Solver()
    client = _client()

    def send(cli, left, right):
        cli.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))

    def on_message(cli, _userdata, msg):
        if msg.topic == TOPIC_SENSORS:
            send(cli, *solver.step(json.loads(msg.payload)))
        elif msg.topic == TOPIC_POSES:
            if msg.payload:
                solver.on_poses(json.loads(msg.payload))
        elif msg.topic == TOPIC_PELLETS:
            if msg.payload:
                solver.on_pellets(json.loads(msg.payload))
        elif msg.topic == TOPIC_RESULT:
            solver.log(f"result: {msg.payload.decode()}")
            solver.done = True
            send(cli, 0.0, 0.0)
            cli.disconnect()

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    for topic in (TOPIC_POSES, TOPIC_PELLETS, TOPIC_RESULT, TOPIC_SENSORS):
        client.subscribe(topic)
    print("connected, waiting for the simulator", flush=True)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass