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
    python3 task_2a_boilerplate.py   # terminal 2
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
# Cells are (row, col), as the maze drawing and pacbot/poses use: row 0 at
# the top, column 0 at the left. As in Task 2B, cell (row, col) is centred
# at world (col * PITCH, (N - 1 - row) * PITCH) - see cell_centre().
EXIT = (N - 1, N - 1, 3)  # the only opening in the border: -y side of (14, 14)

# Directions: 0 = +x, 1 = +y, 2 = -x, 3 = -y (the numbering pacbot/poses uses).
# STEP[d] is the (d_row, d_col) of one cell in direction d: +x is the next
# column, +y the row above.
STEP = ((0, 1), (-1, 0), (0, -1), (1, 0))

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
    """{(row, col, d): True if side d of cell (row, col) is walled}."""
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
    """World (x, y) of the centre of cell (row, col)."""
    return cell[1] * PITCH, (N - 1 - cell[0]) * PITCH


def inside(row, col):
    return 0 <= row < N and 0 <= col < N


def neighbours(cell):
    """Open neighbouring cells of `cell`, as (direction, cell) pairs."""
    row, col = cell
    out = []
    for d in range(4):
        n = (row + STEP[d][0], col + STEP[d][1])
        if not WALLS[row, col, d] and inside(*n):
            out.append((d, n))
    return out


# ----------------------------------------------------------------- robot
WHEEL_R = 0.017           # m, wheel radius
TRACK = 0.078             # m, wheel centre to wheel centre
TICKS_PER_REV = 360       # encoder ticks per wheel revolution
M_PER_TICK = 2 * math.pi * WHEEL_R / TICKS_PER_REV
WHEEL_MAX = 30.0          # rad/s, the simulator's clamp
AXLE_BACK = 0.033         # m, the wheel axle is this far behind the chassis centre
NO_HIT = 2.0              # m, what a ToF reads when nothing is within its 2 m range

# ToF sensors: name -> (x, y) from the chassis centre, and the direction
# the ray points relative to the robot's heading. Note the names:
# "sl" / "sr" face FORWARD; "fl" / "fr" face the SIDES.
SENSORS = (
    ("sl", (-0.005, 0.034), 0.0),
    ("sr", (-0.005, -0.034), 0.0),
    ("fl", (0.036, 0.005), math.pi / 2),
    ("fr", (0.036, -0.005), -math.pi / 2),
)


def wrap(a):
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wheels(v, omega):
    """(left, right) wheel speeds in rad/s for a forward speed v (m/s) and
    a turn rate omega (rad/s, positive = counter-clockwise / left)."""
    left = (v - omega * TRACK / 2) / WHEEL_R
    right = (v + omega * TRACK / 2) / WHEEL_R
    return clamp(left, -WHEEL_MAX, WHEEL_MAX), clamp(right, -WHEEL_MAX, WHEEL_MAX)


# ------------------------------------------------------- controller tuning
V_CRUISE = 0.30        # m/s, straight-line top speed
V_MIN = 0.04           # m/s, creep speed near the stop point
A_ACC = 1.0            # m/s^2, speed-up ramp
A_DEC = 0.8            # m/s^2, braking profile used to pick the approach speed
A_DEC_MAX = 1.5        # m/s^2, fastest allowed slow-down of the command
STOP_TOL = 0.004       # m, a run ends when the axle is this close to its target
KH = 2.5               # heading P gain while driving
KD = 0.30              # heading damping on the turn rate
KL = 3.0               # lateral P gain: rad of heading offset per m off-centre
MAX_REF = 0.20         # rad, cap on that heading offset
OMEGA_MAX = 1.5        # rad/s, cap on the steering command while driving
KP_TURN = 4.0          # pivot P gain
W_TURN_MAX = 3.0       # rad/s, fastest pivot
W_TURN_MIN = 0.3       # rad/s, slowest pivot (avoids stalling)
TURN_TOL = math.radians(1.2)
GYRO_TOL = 0.1         # rad/s, "stopped rotating" threshold
SETTLE_FRAMES = 10     # frames of standing still between actions
HALF_GAP = (PITCH - 2 * WALL_HALF) / 2   # 0.105 m, cell centre to wall face
WALL_NEAR = 0.19       # m, a side reading below this means a wall is there
SIDE_OFF = 0.005       # m, fl/fr sit this far off the centre line
SENSOR_ARM = AXLE_BACK + 0.036   # m, fl/fr ahead of the axle
FRONT_ARM = AXLE_BACK - 0.005    # m, sl/sr ahead of the axle
FRONT_RANGE = 0.8      # m, only trust the front wall reading inside this
LAT_GAIN = 0.10        # per-frame blend of the side-wall position fix
FWD_GAIN = 0.30        # per-frame blend of the front-wall position fix

# Safety layers using raw ToF readings directly
STOP_BACK = 0.018      # m, stop this far BEFORE a wall-ended cell centre
R_HARD = 0.082         # m, front reading at which forward speed is forced to 0
R_ARRIVE = 0.092       # m, front reading that ends a run that faces a wall
SIDE_SAFE = 0.065      # m, start steering away when a side reading drops below this
K_SIDE = 25.0          # rad/s of steering per metre inside SIDE_SAFE

# Turn safety / heading calibration
TURN_TIMEOUT = 8.0     # s, give up a pivot after this long
TURN_RECON = 0.25      # rad, gyro vs encoder heading disagreement that triggers a resync
CAL_ANGLE = 0.30       # rad, size of the startup calibration wiggle
CAL_W = 1.2            # rad/s, wiggle speed


# ============================================================== planning

def plan_route(start, heading, pellet):
    """Plan the route: from `start` facing `heading`, through `pellet`, out
    of the exit."""
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
        if row == N:                 # outside the maze, below the exit
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
    """One sensor frame in, wheel speeds out."""

    def __init__(self):
        self.t = 0.0            # s of sim time since the first sensor frame
        self.spawn = None       # (dir, cell) of the robot at spawn
        self.pellet = None      # (row, col) cell of the pellet
        self.actions = None     # the plan, once made
        self.prev_ticks = None  # (left, right) encoder ticks last frame
        self.done = False
        # Pose of the wheel AXLE midpoint (the point the robot pivots about),
        # in world metres / radians. The chassis centre is AXLE_BACK ahead.
        self.x = self.y = self.th = 0.0
        self.heading = 0        # 0-3, facing at the end of the last action
        self.cell = None        # (row, col) the axle is in / last stopped in
        self.queue = []         # actions still to do (plan + initial align)
        self.phase = "idle"     # idle | run | turn | settle | finished
        self.settle = 0
        self.v_cmd = 0.0
        self.run_c = (0.0, 0.0)
        self.run_th = 0.0
        self.run_cell = None
        self.run_front_wall = False
        self.turn_target = 0.0
        self.turn_dir = 0
        self.ok_count = 0
        self.gyro_bias = 0.0    # rad/s, learned while standing still
        self.track_eff = TRACK  # m, effective track, calibrated against the gyro
        self.gyro_sign = 1.0    # +1 if gyro z > 0 means counter-clockwise
        self.gyro_ok = False    # gyro agrees with the encoders (after calibration)
        self.rate = 0.0         # rad/s, calibrated turn rate
        self.dt = 0.01
        self._dth_enc = 0.0     # this frame's encoder-measured rotation
        self._g_raw = 0.0       # this frame's bias-corrected gyro rotation (raw sign)
        self.turn_enc = 0.0     # encoder rotation since the pivot began
        self.turn_t = 0.0
        self.turn_start_th = 0.0
        self.after_settle = "idle"
        self.cal = None
        self.run_stop = 0.0

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
        # Distance each wheel rolled since the last frame.
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

        # Dead reckoning: gyro for heading, encoders for distance.
        dt = s["dt"]
        self.dt = dt
        if self.phase == "settle" and d_left == 0 and d_right == 0:
            self.gyro_bias += 0.1 * (turn_rate - self.gyro_bias)   # stationary: gyro = bias
        self._g_raw = (turn_rate - self.gyro_bias) * dt
        self._dth_enc = (d_right - d_left) / self.track_eff        # +ve = counter-clockwise

        # Calibrated heading update from Pacbot.txt
        if self.gyro_ok and self.phase != "cal":
            dth = self.gyro_sign * self._g_raw     # gyro, with the sign verified at startup
        else:
            dth = self._dth_enc                    # encoders (no drift or sign ambiguity)

        self.rate = dth / dt
        if self.phase == "turn":
            self.turn_enc += self._dth_enc
            self.turn_t += dt

        mid = self.th + 0.5 * dth
        ds = 0.5 * (d_left + d_right)
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.th += dth
        return self._control(s, turn_rate)

    # ------------------------------------------------------- executor
    def _start(self):
        d, cell = self.spawn
        self.heading = d
        self.cell = tuple(cell)
        self.th = d * math.pi / 2
        cx, cy = cell_centre(cell)
        self.x = cx - AXLE_BACK * math.cos(self.th)
        self.y = cy - AXLE_BACK * math.sin(self.th)
        # ("run", 0) first: drive the axle onto the cell centre so that every
        # later pivot happens about the middle of a cell (clear of the walls).
        self.queue = [("run", 0)] + list(self.actions)
        self.phase = "settle"        # stand still ~0.5 s first to learn gyro bias
        self.settle = 50
        self.after_settle = "cal"    # then a small wiggle to check gyro sign against wheels
        self.v_cmd = 0.0

    def _control(self, s, gyro):
        while self.phase == "idle":
            if not self.queue:
                self.phase = "finished"
                break
            self._begin(self.queue.pop(0))
        if self.phase == "settle":
            self.settle -= 1
            if self.settle <= 0:
                self.phase = self.after_settle
                self.after_settle = "idle"
                if self.phase == "cal":
                    self.cal = dict(stage=0, enc=0.0, gyro=0.0, n=0, t=0.0, th0=self.th)
            return 0.0, 0.0
        if self.phase == "cal":
            return self._calibrate()
        if self.phase == "run":
            return self._run(s)
        if self.phase == "turn":
            return self._turn(gyro)
        return 0.0, 0.0

    def _begin(self, action):
        kind, val = action
        if kind == "turn":
            if val == self.heading:
                return
            self.turn_dir = val
            self.turn_target = self.th + wrap(val * math.pi / 2 - self.th)
            self.ok_count = 0
            self.turn_enc = 0.0
            self.turn_t = 0.0
            self.turn_start_th = self.th
            self.phase = "turn"
            self.log(f"turn to {val}")
        else:
            d = self.heading
            tgt = (self.cell[0] + val * STEP[d][0], self.cell[1] + val * STEP[d][1])
            self.run_cell = tgt
            self.run_c = cell_centre(tgt)
            self.run_th = self.th + wrap(d * math.pi / 2 - self.th)
            self.run_front_wall = inside(*tgt) and WALLS[tgt[0], tgt[1], d]
            # Stop a little short of a wall-ended cell centre to give room to pivot
            self.run_stop = -STOP_BACK if self.run_front_wall else 0.0
            self.v_cmd = 0.0
            self.phase = "run"
            self.log(f"run {val} -> cell {tgt}")

    def _calibrate(self):
        """Wiggle left by CAL_ANGLE using ENCODERS as ground truth, compare
        with the gyro, then pivot back to lock sign and effective track."""
        c = self.cal
        c["t"] += self.dt
        c["enc"] += self._dth_enc
        c["gyro"] += self._g_raw
        if c["stage"] == 0:
            if abs(c["enc"]) >= CAL_ANGLE or c["t"] > 3.0:
                c["stage"], c["n"] = 1, 0
                return 0.0, 0.0
            return wheels(0.0, CAL_W)
        c["n"] += 1
        if c["n"] < 15:                       # let robot coast to a complete stop
            return 0.0, 0.0
        e, g = c["enc"], c["gyro"]
        if abs(g) > 0.08 and abs(e) > 0.15 and e * g != 0:
            ratio = abs(e) / abs(g)
            if 0.6 <= ratio <= 1.6:
                self.gyro_sign = 1.0 if e * g > 0 else -1.0
                self.track_eff = clamp(TRACK * ratio, 0.06, 0.11)
                self.gyro_ok = True
        est = e * (TRACK / self.track_eff) if self.gyro_ok else e
        self.log(f"calibration: enc={e:+.3f} gyro={g:+.3f} sign={self.gyro_sign:+.0f} "
                 f"track={self.track_eff:.4f} gyro_ok={self.gyro_ok}")

        # Reset heading and pivot back to spawn heading
        self.th = c["th0"] + est
        self.turn_target = c["th0"]
        self.turn_dir = self.heading
        self.ok_count = 0
        self.turn_enc = 0.0
        self.turn_t = 0.0
        self.turn_start_th = self.th
        self.phase = "turn"
        return 0.0, 0.0

    def _finish_turn(self):
        self.heading = self.turn_dir
        self.phase = "settle"
        self.settle = SETTLE_FRAMES
        return 0.0, 0.0

    def _turn(self, gyro):
        # Cross-reconcile gyro heading with wheel encoders to kill drift/runaway
        if abs((self.th - self.turn_start_th) - self.turn_enc) > TURN_RECON:
            self.th = self.turn_start_th + self.turn_enc

        err = wrap(self.turn_target - self.th)
        if abs(err) < TURN_TOL and abs(self.rate) < GYRO_TOL:
            self.ok_count += 1
        else:
            self.ok_count = 0
        if self.ok_count >= 5:
            return self._finish_turn()

        # Safety timeouts and encoder limit guards preventing continuous 360 rotation
        planned = abs(wrap(self.turn_target - self.turn_start_th))
        if self.turn_t > TURN_TIMEOUT or abs(self.turn_enc) > planned + 1.0:
            self.log("pivot limit reached; stopping the turn")
            self.th = self.turn_target
            return self._finish_turn()

        if abs(err) < TURN_TOL:
            return 0.0, 0.0

        w = math.copysign(clamp(KP_TURN * abs(err), W_TURN_MIN, W_TURN_MAX), err)
        return wheels(0.0, w)

    def _correct(self, s, ux, uy, nx, ny, eps):
        """Pull the dead-reckoned position toward what the walls say."""
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
            e -= SENSOR_ARM * math.sin(eps)       # sensors are ahead of axle
            cur = (self.x - cx) * nx + (self.y - cy) * ny
            if abs(e - cur) < 0.06:               # reject sensor outlier jumps
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
        nx, ny = -uy, ux                      # left-hand normal of lane
        cx, cy = self.run_c
        eps = wrap(self.th - self.run_th)
        if abs(eps) < 0.15:
            self._correct(s, ux, uy, nx, ny, eps)
        along = (self.x - cx) * ux + (self.y - cy) * uy
        lateral = (self.x - cx) * nx + (self.y - cy) * ny
        rem = self.run_stop - along

        # Raw front ToF check to trigger arrival at wall stop
        r_front = min(s["sl"], s["sr"])
        if self.run_front_wall and r_front <= R_ARRIVE:
            rem = 0.0

        if rem <= STOP_TOL:
            self.x += rem * ux
            self.y += rem * uy
            self.cell = self.run_cell
            if abs(eps) < 0.1:                # snap out heading drift
                self.th = self.run_th
            self.v_cmd = 0.0
            self.phase = "settle"
            self.settle = SETTLE_FRAMES
            return 0.0, 0.0

        v_des = clamp(math.sqrt(2 * A_DEC * rem), V_MIN, V_CRUISE)
        # Independent raw ToF safety braking ceiling
        if r_front < 0.35:
            v_des = min(v_des, math.sqrt(2 * A_DEC * max(r_front - R_HARD, 0.0)))

        if v_des > self.v_cmd:
            self.v_cmd = min(v_des, self.v_cmd + A_ACC * dt)
        else:
            self.v_cmd = max(v_des, self.v_cmd - A_DEC_MAX * dt)

        ref = self.run_th - clamp(KL * lateral, -MAX_REF, MAX_REF)
        omega = KH * wrap(ref - self.th) - KD * self.rate

        # Side collision avoidance push
        fl, fr = s["fl"], s["fr"]
        if fl < SIDE_SAFE:
            omega -= K_SIDE * (SIDE_SAFE - fl)
        if fr < SIDE_SAFE:
            omega += K_SIDE * (SIDE_SAFE - fr)

        omega = clamp(omega, -OMEGA_MAX, OMEGA_MAX)
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