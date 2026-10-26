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
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_POSES = "pacbot/poses"
TOPIC_PELLETS = "pacbot/pellets"
TOPIC_RESULT = "pacbot/result"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

# ------------------------------------------------------------------ maze
N = 15
PITCH = 0.22
WALL_HALF = 0.005
EXIT = (N - 1, N - 1, 3)

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
    "+   +   +   +   +   +   +---+   +   +---+---+   +---+   +",
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
            walls[row, col, 0] = LAYOUT[r][c + 4] == "|"
            walls[row, col, 1] = LAYOUT[r - 1][c + 1:c + 4] == "---"
            walls[row, col, 2] = LAYOUT[r][c] == "|"
            walls[row, col, 3] = LAYOUT[r + 1][c + 1:c + 4] == "---"
    return walls


WALLS = _wall_table()


def cell_centre(cell):
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017
TRACK = 0.078
TICKS_PER_REV = 360
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0
AXLE_BACK = 0.033

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
V_CRUISE = 0.35        # Snappy, purposeful forward speed
V_MIN = 0.06           # Prevent crawling near the stop point
A_ACC = 1.6            # Fast acceleration
A_DEC = 1.6            # Decisive braking
A_DEC_MAX = 2.4
STOP_TOL = 0.008       # Stop cleanly at the cell center

KH = 7.0               # Strong heading P gain to keep straight
KL = 3.0               # Gentle lateral centering gain
MAX_LAT_OMEGA = 0.40   # Max heading bias added for centering (prevents diagonal drift)

KP_TURN = 7.5
KD_TURN = 0.45
W_TURN_MAX = 3.2
W_TURN_MIN = 0.40
TURN_TOL = math.radians(2.0)
GYRO_TOL = 0.15

HALF_GAP = (PITCH - 2 * WALL_HALF) / 2  # 0.105 m
WALL_NEAR = 0.145      # Clear detection of true nearby side walls
SIDE_OFF = 0.005
TARGET_WALL = HALF_GAP - SIDE_OFF       # 0.100 m
SENSOR_ARM = AXLE_BACK + 0.036
FRONT_ARM = AXLE_BACK - 0.005
FRONT_STOP_DIST = HALF_GAP - FRONT_ARM  # 0.077 m


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
        self.settle = 30
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

        if abs(err) < TURN_TOL:
            if abs(omega) < GYRO_TOL:
                self.ok_count += 1
            else:
                self.ok_count = 0
        else:
            self.ok_count = 0

        if self.ok_count >= 2:
            self.heading = self.turn_dir
            self.th = self.turn_target
            self.phase = "settle"
            self.settle = 1
            return 0.0, 0.0

        w = KP_TURN * err - KD_TURN * omega
        w = clamp(w, -W_TURN_MAX, W_TURN_MAX)

        if abs(err) > 0.04 and abs(w) < W_TURN_MIN:
            w = math.copysign(W_TURN_MIN, err)

        return wheels(0.0, w)

    def _run(self, s):
        dt = s["dt"]
        ux, uy = math.cos(self.run_th), math.sin(self.run_th)
        nx, ny = -uy, ux
        cx, cy = self.run_c
        eps = wrap(self.th - self.run_th)

        along = (self.x - cx) * ux + (self.y - cy) * uy
        rem = -along

        # Use front wall measurement to prevent overshoot
        if self.run_front_wall:
            r_front = min(s["sl"], s["sr"]) * math.cos(eps)
            if r_front < 0.40:
                front_rem = r_front - FRONT_STOP_DIST
                rem = max(0.0, front_rem)

        # Stop threshold
        if rem <= STOP_TOL:
            self.x -= along * ux
            self.y -= along * uy
            self.cell = self.run_cell
            self.th = self.run_th  # Guarantee perfectly straight heading before turn
            self.v_cmd = 0.0
            self.phase = "settle"
            self.settle = 1
            return 0.0, 0.0

        # Sensor-based lateral correction only mid-corridor
        fl, fr = s["fl"], s["fr"]
        crow = clamp(int(round(N - 1 - self.y / PITCH)), 0, N - 1)
        ccol = clamp(int(round(self.x / PITCH)), 0, N - 1)
        d = self.heading
        has_left_wall = WALLS.get((crow, ccol, (d + 1) % 4), False)
        has_right_wall = WALLS.get((crow, ccol, (d + 3) % 4), False)

        fl_valid = (fl < WALL_NEAR) and has_left_wall
        fr_valid = (fr < WALL_NEAR) and has_right_wall

        odom_lat = (self.x - cx) * nx + (self.y - cy) * ny
        lat_err = odom_lat

        # Avoid lateral steering near junctions to stay straight
        if rem > 0.06:
            if fl_valid and fr_valid:
                lat_err = 0.5 * (fr - fl) - SENSOR_ARM * math.sin(eps)
            elif fl_valid and abs(eps) < 0.06:
                lat_err = (TARGET_WALL - fl) - SENSOR_ARM * math.sin(eps)
            elif fr_valid and abs(eps) < 0.06:
                lat_err = (fr - TARGET_WALL) - SENSOR_ARM * math.sin(eps)

        lat_err = clamp(lat_err, -0.035, 0.035)

        # Acceleration and deceleration speed profile
        v_des = clamp(math.sqrt(2 * A_DEC * rem), V_MIN, V_CRUISE)
        if v_des > self.v_cmd:
            self.v_cmd = min(v_des, self.v_cmd + A_ACC * dt)
        else:
            self.v_cmd = max(v_des, self.v_cmd - A_DEC_MAX * dt)

        # Steer straight along run_th while softly rejecting lateral drift
        heading_err = wrap(self.run_th - self.th)
        omega = KH * heading_err - clamp(KL * lat_err, -MAX_LAT_OMEGA, MAX_LAT_OMEGA)
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