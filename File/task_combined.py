#!/usr/bin/env python3
"""
PacBot Task 1 (A + B) combined controller.

  1A  - maze knowledge + BFS planning: collect all objectives, then leave
        through the nearest exit.
  1B  - low level motion from sensors: gyro heading hold / in-place turns,
        range-sensor wall detection, trapezoid speed profile, publishing wheel
        velocities on pacbot/wheel_vel.

Run (three terminals):
    mosquitto
    python3 pacbot_sim.py --autostart
    python3 task_combined.py

Topics (same as task_1a.py / task_1b.py):
    in : pacbot/sensors, robot/pose, pellets/pose, bot/cmd
    out: pacbot/wheel_vel  {"left": rad/s, "right": rad/s}
"""
import itertools
import json
import math
from collections import deque

import paho.mqtt.client as mqtt

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"
POSE_TOPIC = "robot/pose"
PELLETS_TOPIC = "pellets/pose"
BOT_CMD_TOPIC = "bot/cmd"

# ----------------------------------------------------------------------------
# Geometry  (must match pacbot_sim.py)
# ----------------------------------------------------------------------------
WHEEL_R = 0.017
TRACK = 0.092
CELL = 0.22               # maze cell pitch [m]   (pacbot_sim.py --cell, reference = 0.22)
WALL_T = 0.012
AXLE_TO_SENSOR = 0.028    # forward-looking beams (JSON "sl","sr") sit this far ahead of the axle
# distance those beams read when the AXLE is exactly at the cell centre and a
# wall closes the cell ahead
FRONT_AT_CENTER = CELL / 2 - WALL_T / 2 - AXLE_TO_SENSOR

# ----------------------------------------------------------------------------
# Tunables (names/values follow task_1b.py where they exist)
# ----------------------------------------------------------------------------
BASE_SPEED = 22.0          # cruise wheel speed [rad/s]  (motors saturate at 30)
MIN_APPROACH_SPEED = 1.5   # creep floor near the target [rad/s]
ACCEL = 220.0              # wheel accel limit [rad/s^2]
DECEL = 160.0              # wheel decel used by the stopping profile [rad/s^2]
STOP_TOL = 0.003           # stop when this close to the target [m]
SETTLE_TICKS = 15          # ticks of standing still between moves

TURN_RATE_MAX = 4.5        # body yaw rate [rad/s]
TURN_RATE_MIN = 1.0
TURN_KP = 6.0
TURN_TOL = math.radians(1.5)

K_HEAD = 8.0               # heading hold (rad/s of wheel speed per rad)
K_GYRO = 0.7
K_LAT = 2.0                # lateral centring: rad of heading offset per metre
LAT_MAX = math.radians(6)
U_LIMIT = 3.0
SIDE_VALID = 0.20          # side beam counts as "wall present" below this

# ----------------------------------------------------------------------------
# Maze (task_1a.py)
# ----------------------------------------------------------------------------
MAZE_ROWS = 13
MAZE_COLS = 13
WALL_N, WALL_E, WALL_S, WALL_W = 0x1, 0x2, 0x4, 0x8
WALLS = [
    [12, 6, 12, 6, 13, 4, 0, 4, 5, 6, 12, 5, 6],
    [10, 11, 10, 10, 12, 3, 8, 2, 12, 1, 1, 6, 10],
    [8, 5, 3, 8, 1, 6, 9, 2, 9, 6, 13, 2, 10],
    [10, 12, 4, 3, 12, 1, 4, 0, 6, 9, 4, 2, 10],
    [10, 10, 8, 5, 3, 13, 2, 10, 9, 6, 10, 11, 10],
    [8, 3, 9, 4, 5, 6, 8, 1, 6, 10, 9, 5, 2],
    [10, 12, 4, 1, 6, 10, 9, 6, 10, 8, 5, 5, 2],
    [8, 1, 2, 12, 3, 10, 12, 3, 9, 2, 12, 6, 10],
    [9, 6, 10, 10, 12, 1, 2, 12, 5, 1, 0, 1, 3],
    [14, 8, 1, 1, 3, 12, 1, 3, 12, 4, 2, 12, 6],
    [8, 0, 4, 7, 12, 3, 12, 6, 10, 9, 1, 2, 10],
    [10, 10, 9, 6, 10, 12, 2, 8, 1, 7, 12, 0, 2],
    [9, 1, 5, 1, 1, 3, 8, 1, 5, 5, 3, 9, 3],
]
EXIT_CELLS = [(0, 6, 'south'), (MAZE_ROWS - 1, 6, 'north')]
FACING_DEG = {'east': 0.0, 'north': 90.0, 'west': 180.0, 'south': 270.0}
# (dr, dc, wall bit, heading in degrees)   0=EAST 90=NORTH 180=WEST 270=SOUTH
MOVES = [(0, 1, WALL_E, 0.0), (1, 0, WALL_N, 90.0),
         (0, -1, WALL_W, 180.0), (-1, 0, WALL_S, 270.0)]
EXIT_OUT = {(r, c): (r + (-1 if f == 'south' else 1), c, FACING_DEG[f]) for r, c, f in EXIT_CELLS}


# ============================================================================
# 1A part: planning
# ============================================================================
def bfs(src):
    """Shortest paths from src. Returns (dist, parent)."""
    dist, par = {src: 0}, {src: None}
    q = deque([src])
    while q:
        cur = q.popleft()
        r, c = cur
        for dr, dc, bit, _ in MOVES:
            if WALLS[r][c] & bit:
                continue
            n = (r + dr, c + dc)
            if 0 <= n[0] < MAZE_ROWS and 0 <= n[1] < MAZE_COLS and n not in dist:
                dist[n], par[n] = dist[cur] + 1, cur
                q.append(n)
    return dist, par


def path_to(par, dst):
    p = []
    while dst is not None:
        p.append(dst)
        dst = par[dst]
    return p[::-1]


def plan_path(cell, pellets):
    """Cell path (ending one cell OUTSIDE the maze when leaving) towards the
    best next goal. With pellets left: best visiting order of all pellets, then
    the closest exit.  Without: straight to the closest exit."""
    exits = [(r, c) for r, c, _ in EXIT_CELLS]
    dist, par = bfs(cell)
    if not pellets:
        reach = [e for e in exits if e in dist]
        if not reach:
            return None
        e = min(reach, key=lambda x: dist[x])
        p = path_to(par, e)
        orow, ocol, _ = EXIT_OUT[e]
        return p + [(orow, ocol)]
    pts = list(pellets)
    dcache = {p: bfs(p)[0] for p in pts}
    dcache[cell] = dist
    if len(pts) <= 6:
        best, best_first = None, None
        for order in itertools.permutations(pts):
            tot, cur, ok = 0, cell, True
            for p in order:
                if p not in dcache[cur]:
                    ok = False
                    break
                tot += dcache[cur][p]
                cur = p
            if not ok:
                continue
            tot += min(dcache[cur].get(e, 10 ** 6) for e in exits)
            if best is None or tot < best:
                best, best_first = tot, order[0]
        first = best_first
    else:
        first = min(pts, key=lambda p: dist.get(p, 10 ** 6))
    if first is None or first not in dist:
        return None
    return path_to(par, first)


def heading_of(a, b):
    dr, dc = b[0] - a[0], b[1] - a[1]
    for mr, mc, _, deg in MOVES:
        if (mr, mc) == (dr, dc):
            return deg
    return None


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ============================================================================
# 1B part: motion state machine (one call per sensor message)
# ============================================================================
class Controller:
    def __init__(self):
        self.running = False
        self.cell = None
        self.yaw_deg = None
        self.pellets = None
        self.mode = 'WAIT'
        self.psi = None            # absolute heading from gyro [rad]
        self.psi_target = 0.0
        self.travel = 0.0
        self.odo = 0.0             # distance driven in the current run [m]
        self.run_start = (0, 0)    # cell where the current run began
        self.run_dir = None        # (dr, dc) of the current run
        self.run_k = 0             # cells already crossed in this run
        self.v_prev = 0.0
        self.settle = 0
        self.tick = 0
        self.leaving = False
        self.last_log = None

    # ---- inputs from the other topics -------------------------------------
    def on_pose(self, d):
        # wire is swapped (see task_1a.py): "col" carries the grid row
        self.cell = (int(d["col"]), int(d["row"]))
        self.yaw_deg = float(d.get("yaw", 0.0)) % 360.0
        # Landmark: while driving, a new cell in the pose means the axle has just
        # crossed a cell boundary -> re-anchor the odometry (cancels wheel slip).
        if self.mode == 'ADVANCE' and self.run_dir is not None:
            k = ((self.cell[0] - self.run_start[0]) * self.run_dir[0]
                 + (self.cell[1] - self.run_start[1]) * self.run_dir[1])
            if k > self.run_k:
                self.run_k = k
                self.odo = (k - 0.5) * CELL + 0.002     # +2 mm ~ pose message latency

    def on_pellets(self, payload):
        self.pellets = {tuple(c) for c in payload}

    def on_cmd(self, text):
        self.running = text.startswith("1")

    # ---- helpers --------------------------------------------------------------
    def log(self, msg):
        if msg != self.last_log:
            print(msg)
            self.last_log = msg

    def plan(self):
        """Pick the next straight run. Returns False if there is nothing to do."""
        if self.cell is None or self.pellets is None:
            return False
        r, c = self.cell
        if not (0 <= r < MAZE_ROWS and 0 <= c < MAZE_COLS):
            return False                       # already outside the maze
        path = plan_path(self.cell, self.pellets)
        if path is None or len(path) < 2:
            return False
        h = heading_of(path[0], path[1])
        k = 1
        while k + 1 < len(path) and heading_of(path[k], path[k + 1]) == h:
            k += 1
        self.leaving = not self.pellets and k >= 1 and not (
            0 <= path[-1][0] < MAZE_ROWS and 0 <= path[-1][1] < MAZE_COLS)
        # absolute target heading: nearest equivalent of h to the current psi
        err = wrap_pi(math.radians(h) - self.psi)
        self.psi_target = self.psi + err
        self.travel = k * CELL
        self.run_start = self.cell
        self.run_dir = next((m[0], m[1]) for m in MOVES if m[3] == h)
        self.run_k = 0
        self.log(f"[plan] {self.cell} -> heading {h:.0f} deg, {k} cell(s), "
                 f"pellets left={len(self.pellets)}{'  (EXIT run)' if self.leaving else ''}")
        return True

    # ---- main tick ----------------------------------------------------------------
    def on_sensors(self, d):
        """Returns (left_vel, right_vel) in rad/s."""
        self.tick += 1
        dt = float(d["dt"]) if d.get("dt") and d["dt"] > 0 else 0.002
        # Same key meaning as task_1b.py: "sl","sr" (tof_side_*) look straight
        # ahead -> wall distance; "fl","fr" (tof_front_*) look left / right.
        front = min(float(d["sl"]), float(d["sr"]))
        side_l, side_r = float(d["fl"]), float(d["fr"])
        yaw_rate = float(d["gyro"][2])
        enc_l, enc_r = d["enc"]            # wheel angular velocities [rad/s]

        if self.psi is None:
            if self.yaw_deg is None:
                return 0.0, 0.0
            self.psi = math.radians(self.yaw_deg)
        self.psi += yaw_rate * dt

        left = right = 0.0

        if self.mode == 'WAIT':
            if self.running and self.cell is not None and self.pellets is not None:
                self.mode = 'PLAN'

        elif self.mode == 'PLAN':
            if not self.running:
                self.mode = 'WAIT'
            elif self.plan():
                self.v_prev = 0.0
                self.mode = 'TURN' if abs(self.psi_target - self.psi) > TURN_TOL else 'ADVANCE'
                self.odo = 0.0
            else:
                self.mode = 'DONE'
                self.log("[done] nothing left to do")

        elif self.mode == 'TURN':
            err = self.psi_target - self.psi
            if abs(err) <= TURN_TOL:
                self.mode = 'ADVANCE'
                self.odo = 0.0
                self.v_prev = 0.0
            else:
                w = math.copysign(clamp(TURN_KP * abs(err), TURN_RATE_MIN, TURN_RATE_MAX), err)
                ws = w * (TRACK / 2.0) / WHEEL_R
                left, right = -ws, ws

        elif self.mode == 'ADVANCE':
            self.odo += WHEEL_R * (enc_l + enc_r) / 2.0 * dt
            rem = self.travel - self.odo
            if front < 0.6:                    # a wall ahead gives a better reference
                rem = min(rem, front - FRONT_AT_CENTER)
            if rem <= STOP_TOL:
                self.mode = 'SETTLE'
                self.settle = 0
            else:
                v_prof = math.sqrt(2.0 * DECEL * rem / WHEEL_R)
                v = clamp(v_prof, MIN_APPROACH_SPEED, BASE_SPEED)
                v = min(v, max(self.v_prev, MIN_APPROACH_SPEED) + ACCEL * dt)
                self.v_prev = v
                # heading hold, with a gentle pull towards the corridor centre
                target = self.psi_target
                if side_l < SIDE_VALID and side_r < SIDE_VALID and rem > 0.04:
                    target += clamp(K_LAT * (side_l - side_r) / 2.0, -LAT_MAX, LAT_MAX)
                steer = K_HEAD * (target - self.psi) - K_GYRO * yaw_rate
                steer = clamp(steer, -U_LIMIT, U_LIMIT)
                left, right = v - steer, v + steer

        elif self.mode == 'SETTLE':
            self.settle += 1
            if self.settle >= SETTLE_TICKS:
                self.mode = 'PLAN'

        # DONE -> zeros
        if self.tick % 250 == 0 and self.mode != 'DONE':
            print(f"[{self.mode:7s}] cell={self.cell} yaw={math.degrees(self.psi):+7.1f} "
                  f"front={front:.3f} sides=({side_l:.3f},{side_r:.3f}) "
                  f"L={left:+5.1f} R={right:+5.1f}")
        return left, right


# ============================================================================
def _mqtt_client():
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")
    return mqtt.Client(client_id="Controller")


def main():
    ctl = Controller()
    client = _mqtt_client()

    def on_message(cl, userdata, msg):
        try:
            if msg.topic == TOPIC_SENSORS:
                left, right = ctl.on_sensors(json.loads(msg.payload.decode()))
                cl.publish(TOPIC_WHEEL_VEL, json.dumps({"left": float(left), "right": float(right)}))
            elif msg.topic == POSE_TOPIC:
                ctl.on_pose(json.loads(msg.payload.decode()))
            elif msg.topic == PELLETS_TOPIC:
                ctl.on_pellets(json.loads(msg.payload.decode()))
            elif msg.topic == BOT_CMD_TOPIC:
                ctl.on_cmd(msg.payload.decode())
        except Exception as e:                       # keep running on a bad packet
            print("[controller] error:", repr(e))

    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.subscribe([(TOPIC_SENSORS, 0), (POSE_TOPIC, 0), (PELLETS_TOPIC, 0), (BOT_CMD_TOPIC, 0)])
    print("[controller] ready - publish '1' on bot/cmd (or start the sim with --autostart)")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
