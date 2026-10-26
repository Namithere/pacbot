#!/usr/bin/env python3
"""
PacBot MuJoCo maze simulator  (drop-in for ./task_1b_launch, with objectives)

Robot, sensors, motors, timestep, colours and JSON formats are taken from the
MJCF that task_1b_launch embeds (chassis_sim.stl + roda_sim.stl, rear axle,
front caster, 4 ToF rangefinders, MPU6050 gyro/accel, velocity motors).
The maze is the 13x13 WALLS table of task_1a.py (row 0 = SOUTH, col 0 = WEST,
world +x = EAST, +y = NORTH, origin = SW corner).  Interior walls are red,
border walls dark grey, as in the reference scene.

Objectives: two gold pellets are placed in the maze.  Drive the robot's axle
centre over them to capture them (pellets/pose is updated).  Then leave through
one of the two exits (north / south, yellow bars).  Leaving with objectives
still uncaptured does not count as solved.

Run (three terminals):
    mosquitto
    python3 pacbot_sim.py --autostart
    python3 task_combined.py

MQTT interface
--------------
 publishes  pacbot/sensors  {"fl","fr","sl","sr","gyro":[x,y,z],"accel":[x,y,z],"dt",
                             "t","enc":[l,r]}                 (identical keys to task_1b_launch)
               keys are the SENSOR names of the reference model:
               fl/fr = tof_front_left/right : at the front corners, look sideways (+y/-y)
               sl/sr = tof_side_left/right  : at the widest point, look straight ahead (+x)
               (that is why task_1b.py maps  fl=data["sl"], sl=data["fl"])
               gyro [rad/s] body rates (z = yaw rate, CCW +), accel [m/s^2] specific force,
               enc = wheel angular velocities [rad/s] (the model's jointvel encoders; extra key)
 subscribes pacbot/wheel_vel {"left": rad/s, "right": rad/s}   (clipped to +-30)
 publishes  robot/pose      {"row","col","yaw"}  (task_1a.py convention: wire is swapped,
               "col" carries the grid ROW, "row" the grid COL; yaw snapped to 0/90/180/270)
 publishes  pellets/pose    [[row,col],...]      objectives still on the floor (retained)
 publishes  pacbot/result   {"solved","collisions","time_sec"}  when the bot leaves the maze
 publishes  sim/status, sim/truth (debug)
 subscribes bot/cmd         "1" start / "0" stop  (--autostart publishes a retained "1")
 subscribes sim/reset       any payload -> respawn
Robot reference point for the grid pose / pellets is the AXLE CENTRE (the point the
bot spins around).
"""
import argparse
import json
import math
import os
import random
import sys
import threading
import time
from collections import deque

import numpy as np
import mujoco
import paho.mqtt.client as mqtt

# ----------------------------------------------------------------------------
# Maze (identical to task_1a.py)
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
DIRS = [(1, 0, WALL_N), (0, 1, WALL_E), (-1, 0, WALL_S), (0, -1, WALL_W)]

# ----------------------------------------------------------------------------
# Constants (robot values from the task_1b_launch reference model)
# ----------------------------------------------------------------------------
TIMESTEP = 0.002            # s  (== "dt" in the sensor message)
WHEEL_R = 0.017             # m
AXLE_X = -0.033             # m  axle position in the body frame (rear axle)
WHEEL_Y = 0.039             # m  wheel offset from the centre line
BODY_Z = 0.0175             # m  body-origin height (wheel radius + 0.5 mm)
WALL_T = 0.012              # m
WALL_H = 0.060              # m
SENSOR_RANGE = 2.0          # m  reported when a beam hits nothing
PELLET_RADIUS = 0.07        # m  capture radius around the pellet (axle centre)
MAX_WHEEL = 30.0            # rad/s (ctrlrange of the reference motors)
CMD_TIMEOUT = 0.30          # s  wheels stop if no wheel_vel arrives

# ToF rangefinders: body-frame position (z relative to body origin) and beam
# direction -- exactly the sites of the reference model.
SENSORS = {
    'fl': ((0.036, +0.005, -0.001), (0.0, +1.0, 0.0)),    # tof_front_left  (looks left)
    'fr': ((0.036, -0.005, -0.001), (0.0, -1.0, 0.0)),    # tof_front_right (looks right)
    'sl': ((-0.005, +0.034, -0.001), (1.0, 0.0, 0.0)),    # tof_side_left   (looks ahead)
    'sr': ((-0.005, -0.034, -0.001), (1.0, 0.0, 0.0)),    # tof_side_right  (looks ahead)
}

# qpos layout: 0..6 free joint | 7 left wheel | 8 right wheel | pellets ...
Q_WL, Q_WR, Q_FIRST_PELLET = 7, 8, 9


# ----------------------------------------------------------------------------
# MJCF
# ----------------------------------------------------------------------------
def wall_edges():
    """Unique wall segments.  ('H', row_line, col): y = row_line*cell ;
    ('V', row, col_line): x = col_line*cell."""
    edges = set()
    for r in range(MAZE_ROWS):
        for c in range(MAZE_COLS):
            w = WALLS[r][c]
            if w & WALL_N: edges.add(('H', r + 1, c))
            if w & WALL_S: edges.add(('H', r, c))
            if w & WALL_E: edges.add(('V', r, c + 1))
            if w & WALL_W: edges.add(('V', r, c))
    return sorted(edges)


def cell_center(rc, cell):
    return ((rc[1] + 0.5) * cell, (rc[0] + 0.5) * cell)


def build_mjcf(cell, pellets, start):
    S, T, H = cell, WALL_T, WALL_H
    W, Hm = MAZE_COLS * S, MAZE_ROWS * S
    cx, cy = W / 2, Hm / 2
    yaw = math.radians(start[2])
    ax, ay = cell_center(start[:2], S)                       # axle centre = cell centre
    bx, by = ax - AXLE_X * math.cos(yaw), ay - AXLE_X * math.sin(yaw)
    L = []
    a = L.append
    a('<mujoco model="micromouse_maze">')
    a('<compiler angle="degree" autolimits="true"/>')
    a(f'<option timestep="{TIMESTEP}" integrator="implicitfast"/>')
    a(f'<statistic center="{cx:.3f} {cy:.3f} 0.02" extent="{max(W, Hm) * 0.6:.2f}" meansize="0.01"/>')
    a('<visual><map znear="0.01" zfar="50"/><quality shadowsize="4096"/>'
      '<headlight ambient="0.25 0.25 0.25" diffuse="0.4 0.4 0.4" specular="0.1 0.1 0.1"/></visual>')
    a('<asset>')
    a('<texture name="mat" type="2d" builtin="checker" mark="random" rgb1="0.52 0.34 0.18" '
      'rgb2="0.43 0.27 0.13" markrgb="0.58 0.4 0.22" random="0.05" width="512" height="512"/>')
    a('<material name="mat" texture="mat" texrepeat="1 16" reflectance="0.15" shininess="0.3"/>')
    a('<mesh name="chassis" file="chassis_sim.stl" scale="0.001 0.001 0.001"/>')
    a('<mesh name="roda" file="roda_sim.stl" scale="1 1 1"/>')
    a('</asset>')
    a('<worldbody>')
    a(f'<light name="top" pos="{cx:.3f} {cy:.3f} 1.2" dir="0 0 -1" diffuse="0.9 0.9 0.9" specular="0.3 0.3 0.3" castshadow="true"/>')
    a(f'<light name="fill_sw" pos="0 0 0.6" dir="0.35 0.35 -1" diffuse="0.35 0.35 0.4" castshadow="false"/>')
    a(f'<light name="fill_ne" pos="{W:.3f} {Hm:.3f} 0.6" dir="-0.35 -0.35 -1" diffuse="0.35 0.35 0.4" castshadow="false"/>')
    a('<geom name="floor" type="plane" size="0 0 0.05" material="mat" friction="1.0 0.005 0.0001"/>')

    # ---- walls: interior red, border dark grey (as in the reference scene)
    def box(name, x, y, hx, hy, rgba):
        a(f'<geom name="{name}" type="box" pos="{x:.5f} {y:.5f} {H / 2}" size="{hx:.5f} {hy:.5f} {H / 2}" '
          f'contype="1" conaffinity="1" rgba="{rgba}"/>')
    RED, DARK = "0.85 0.25 0.2 1", "0.3 0.3 0.32 1"
    for k, (kind, i, j) in enumerate(wall_edges()):
        border = (i in (0, MAZE_ROWS) and kind == 'H') or (j in (0, MAZE_COLS) and kind == 'V')
        if kind == 'H':
            box(f'wall_h{k}', (j + 0.5) * S, i * S, S / 2 + T / 2, T / 2, DARK if border else RED)
        else:
            box(f'wall_v{k}', j * S, (i + 0.5) * S, T / 2, S / 2 + T / 2, DARK if border else RED)
    # exit chutes: keep a wall on each side as the bot drives out
    chute = 0.9 * S
    for (r, c, facing) in EXIT_CELLS:
        y0 = 0.0 if facing == 'south' else Hm
        sgn = -1 if facing == 'south' else 1
        for side, xl in (('a', c * S), ('b', (c + 1) * S)):
            box(f'wall_exit_{facing}_{side}', xl, y0 + sgn * chute / 2, T / 2, chute / 2, DARK)
    # ---- markers (flat, non-colliding): green = start, yellow = exits
    sxm, sym = cell_center(start[:2], S)
    a(f'<geom name="mark_start" type="box" pos="{sxm:.4f} {sym:.4f} 0.001" size="0.09 0.02 0.001" '
      f'contype="0" conaffinity="0" group="2" rgba="0.2 0.8 0.2 1"/>')
    for (r, c, facing) in EXIT_CELLS:
        y0 = 0.0 if facing == 'south' else Hm
        a(f'<geom name="mark_exit_{facing}" type="box" pos="{(c + 0.5) * S:.4f} {y0:.4f} 0.001" '
          f'size="0.09 0.02 0.001" contype="0" conaffinity="0" group="2" rgba="0.9 0.8 0.1 1"/>')

    # ---- robot (reference model) ------------------------------------------------
    a(f'<body name="base" pos="{bx:.5f} {by:.5f} {BODY_Z}" euler="0 0 {math.degrees(yaw):.4f}">')
    a('<freejoint name="root"/>')
    a('<inertial pos="-0.015 0 0.005" mass="0.20" diaginertia="0.0002 0.0003 0.0004"/>')
    a('<geom name="shell" type="mesh" mesh="chassis" pos="0 0 -0.017" euler="0 0 -90" '
      'contype="0" conaffinity="0" group="2" rgba="0.2 0.5 0.85 1"/>')
    a('<geom name="chassis_col" type="box" size="0.046 0.043 0.011" pos="0 0 0.001" '
      'contype="1" conaffinity="1" group="3" rgba="0 0 0 0"/>')
    for side, ys in (('left', 1), ('right', -1)):
        a(f'<body name="{side}_wheel" pos="{AXLE_X} {ys * WHEEL_Y} 0">')
        a(f'<joint name="{side}_wheel_joint" type="hinge" axis="0 1 0"/>')
        a(f'<geom type="cylinder" size="{WHEEL_R} 0.0045" euler="90 0 0" friction="2.0 0.005 0.0001" '
          f'contype="1" conaffinity="1" group="3" rgba="0.1 0.1 0.1 0"/>')
        a(f'<geom type="mesh" mesh="roda" euler="0 0 {90 * ys}" contype="0" conaffinity="0" '
          f'group="2" rgba="0.15 0.15 0.15 1"/>')
        a('</body>')
    a('<body name="caster" pos="0.038 0 -0.011">')
    a('<geom type="sphere" size="0.006" friction="0.005 0.001 0.0001" contype="1" conaffinity="1" '
      'group="3" rgba="0.6 0.6 0.6 0"/>')
    a('</body>')
    a('</body>')

    # ---- objectives: purely visual gold pellets that sink below the floor when taken
    for k, rc in enumerate(pellets):
        x, y = cell_center(rc, S)
        a(f'<body name="pellet{k}" pos="{x:.4f} {y:.4f} 0.02" gravcomp="1">')
        a(f'<joint name="pellet{k}" type="slide" axis="0 0 1" damping="100"/>')
        a('<inertial pos="0 0 0" mass="0.001" diaginertia="1e-8 1e-8 1e-8"/>')
        a('<geom type="sphere" size="0.012" rgba="1 0.78 0.1 1" contype="0" conaffinity="0" group="2"/>')
        a('</body>')
    a('</worldbody>')
    a('<actuator>')
    a(f'<velocity name="left_motor" joint="left_wheel_joint" kv="0.05" ctrlrange="-{MAX_WHEEL:g} {MAX_WHEEL:g}" forcerange="-0.02 0.02"/>')
    a(f'<velocity name="right_motor" joint="right_wheel_joint" kv="0.05" ctrlrange="-{MAX_WHEEL:g} {MAX_WHEEL:g}" forcerange="-0.02 0.02"/>')
    a('</actuator>')
    a('</mujoco>')
    return '\n'.join(L)


def load_assets(asset_dir):
    assets = {}
    for name in ('chassis_sim.stl', 'roda_sim.stl'):
        p = os.path.join(asset_dir, name)
        if not os.path.isfile(p):
            sys.exit(f'[sim] missing mesh {p}  (use --assets DIR)')
        with open(p, 'rb') as f:
            assets[name] = f.read()
    return assets


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
def bfs_dist(src):
    dist = {src: 0}
    q = deque([src])
    while q:
        r, c = q.popleft()
        for dr, dc, bit in DIRS:
            if not WALLS[r][c] & bit:
                n = (r + dr, c + dc)
                if 0 <= n[0] < MAZE_ROWS and 0 <= n[1] < MAZE_COLS and n not in dist:
                    dist[n] = dist[(r, c)] + 1
                    q.append(n)
    return dist


def pick_pellets(start, n, seed):
    rng = random.Random(seed)
    d0 = bfs_dist(start)
    exits = {(r, c) for r, c, _ in EXIT_CELLS}
    cand = [c for c, d in d0.items() if d >= 5 and c not in exits]
    for _ in range(1000):
        pick = rng.sample(cand, n)
        if all(bfs_dist(a).get(b, 0) >= 5 for i, a in enumerate(pick) for b in pick[i + 1:]):
            return pick
    return rng.sample(cand, n)


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mqtt_client():
    if hasattr(mqtt, 'CallbackAPIVersion'):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id='PacBotSim')
    return mqtt.Client(client_id='PacBotSim')


# ----------------------------------------------------------------------------
# Simulator
# ----------------------------------------------------------------------------
class PacBotSim:
    def __init__(self, args):
        self.args = args
        self.S = args.cell
        sr, sc, sy = (float(v) for v in args.start.split(','))
        self.start = (int(sr), int(sc), sy)
        if args.pellets:
            self.pellet_cells = [tuple(int(v) for v in p.split(',')) for p in args.pellets.split()]
        else:
            self.pellet_cells = pick_pellets(self.start[:2], 2, args.seed)
        self.n_pellets = len(self.pellet_cells)

        xml = build_mjcf(self.S, self.pellet_cells, self.start)
        self.model = mujoco.MjModel.from_xml_string(xml, load_assets(args.assets))
        self.data = mujoco.MjData(self.model)
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'base')
        assert self.model.nq == Q_FIRST_PELLET + self.n_pellets, 'unexpected qpos layout'
        self.q_pellet = [Q_FIRST_PELLET + k for k in range(self.n_pellets)]
        self.rng = np.random.default_rng(1)
        self.ray_groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)   # group 0 = walls only
        self.geomid = np.zeros(1, dtype=np.int32)
        self.ray_normal = np.zeros(3)

        # collision counting (needs the contact list of the real mujoco bindings)
        self.col_id = -1
        self.wall_ids = set()
        if hasattr(self.data, 'ncon'):
            gid = lambda n: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, n)
            self.col_id = gid('chassis_col')
            self.wall_ids = {i for i in range(self.model.ngeom)
                             if (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) or '').startswith('wall')}

        self.cmd_l = self.cmd_r = 0.0
        self.last_cmd_t = -1e9
        self.cmd_evt = threading.Event()
        self.reset_req = False
        self.reset()

    # -- state -----------------------------------------------------------------
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        yaw = math.radians(self.start[2])
        ax, ay = cell_center(self.start[:2], self.S)
        q = self.data.qpos
        q[0:3] = (ax - AXLE_X * math.cos(yaw), ay - AXLE_X * math.sin(yaw), BODY_Z)
        q[3:7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))
        for i in range(self.n_pellets):
            q[self.q_pellet[i]] = 0.0
        self.collected = [False] * self.n_pellets
        self.escaped = False
        self.collisions = 0
        self.in_contact = False
        self.last_cell = self.last_yaw = None
        self.prev_v = np.zeros(3)
        self.cmd_l = self.cmd_r = 0.0
        mujoco.mj_forward(self.model, self.data)

    def pose(self):
        """Axle-centre position and yaw."""
        q = self.data.qpos
        w, qx, qy, qz = (float(v) for v in q[3:7])
        yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        x = float(q[0]) + AXLE_X * math.cos(yaw)
        y = float(q[1]) + AXLE_X * math.sin(yaw)
        return x, y, yaw

    def cell_and_yaw(self):
        x, y, yaw = self.pose()
        cell = (int(math.floor(y / self.S)), int(math.floor(x / self.S)))
        snapped = (int(round(math.degrees(yaw) / 90.0)) % 4) * 90.0
        return cell, snapped

    def pellets_left(self):
        return [list(self.pellet_cells[i]) for i in range(self.n_pellets) if not self.collected[i]]

    # -- sensors -----------------------------------------------------------------
    def _ray(self, p, v):
        # mujoco >= 3.3 added a `normal` output argument to mj_ray; support both.
        try:
            return mujoco.mj_ray(self.model, self.data, p, v, self.ray_groups, 1,
                                 self.body_id, self.geomid)
        except TypeError:
            return mujoco.mj_ray(self.model, self.data, p, v, self.ray_groups, 1,
                                 self.body_id, self.geomid, self.ray_normal)

    def range_sensors(self):
        q = self.data.qpos
        pos = np.array(q[0:3], dtype=np.float64)
        R = quat_to_mat(q[3:7])
        out = {}
        for name, (off, d) in SENSORS.items():
            dist = self._ray(pos + R @ np.array(off), R @ np.array(d))
            if dist < 0 or dist > SENSOR_RANGE:
                dist = SENSOR_RANGE
            if self.args.noise > 0:
                dist = max(0.0, dist + self.rng.normal(0, 0.002 * self.args.noise))
            out[name] = dist
        return out

    def sensor_msg(self, dt):
        r = self.range_sensors()
        q, qd = self.data.qpos, self.data.qvel
        gyro = np.array(qd[3:6], dtype=np.float64)
        R = quat_to_mat(q[3:7])
        v = np.array(qd[0:3], dtype=np.float64)
        acc_w = (v - self.prev_v) / dt
        self.prev_v = v
        accel = R.T @ (acc_w + np.array([0.0, 0.0, 9.81]))       # specific force, body frame
        if self.args.noise > 0:
            gyro = gyro + self.rng.normal(0, 0.003 * self.args.noise, 3)
            accel = accel + self.rng.normal(0, 0.05 * self.args.noise, 3)
        return {
            'fl': round(r['fl'], 6), 'fr': round(r['fr'], 6),
            'sl': round(r['sl'], 6), 'sr': round(r['sr'], 6),
            'gyro': [round(float(g), 6) for g in gyro],
            'accel': [round(float(a), 6) for a in accel],
            'dt': dt,
            'enc': [round(float(qd[6]), 5), round(float(qd[7]), 5)],     # extra: wheel rad/s
            't': round(float(self.data.time), 5),
        }

    # -- game logic --------------------------------------------------------------
    def count_collisions(self):
        if self.col_id < 0:
            return
        d = self.data
        hit = False
        for i in range(d.ncon):
            c = d.contact[i]
            if (c.geom1 == self.col_id and c.geom2 in self.wall_ids) or \
               (c.geom2 == self.col_id and c.geom1 in self.wall_ids):
                hit = True
                break
        if hit and not self.in_contact:
            self.collisions += 1
        self.in_contact = hit

    def update_game(self, client):
        x, y, _ = self.pose()
        changed = False
        for i, rc in enumerate(self.pellet_cells):
            if not self.collected[i]:
                px, py = cell_center(rc, self.S)
                if math.hypot(x - px, y - py) < PELLET_RADIUS:
                    self.collected[i] = True
                    self.data.qpos[self.q_pellet[i]] = -0.5          # sink below the floor
                    changed = True
                    print(f'[sim] t={self.data.time:6.2f}s objective captured at cell {rc} '
                          f'({sum(self.collected)}/{self.n_pellets})')
        if changed:
            self.publish_pellets(client)
        W, Hm = MAZE_COLS * self.S, MAZE_ROWS * self.S
        if not self.escaped and not (0 <= x <= W and 0 <= y <= Hm):
            self.escaped = True
            solved = all(self.collected)
            t = float(self.data.time)
            print(f'[sim] *** bot left the maze at t={t:.2f}s  '
                  f'{"MAZE SOLVED" if solved else "NOT SOLVED - objectives missing"}  '
                  f'collisions={self.collisions} ***')
            client.publish('pacbot/result', json.dumps(
                {'solved': solved, 'collisions': self.collisions, 'time_sec': round(t, 2)}))

    def publish_pellets(self, client):
        client.publish('pellets/pose', json.dumps(self.pellets_left()), retain=True)

    def publish_pose(self, client, force=False):
        cell, yaw = self.cell_and_yaw()
        if force or cell != self.last_cell or yaw != self.last_yaw:
            self.last_cell, self.last_yaw = cell, yaw
            # wire is swapped (see task_1a.py): "col" <- grid row, "row" <- grid col
            client.publish('robot/pose', json.dumps({'row': cell[1], 'col': cell[0], 'yaw': yaw}))

    # -- mqtt -------------------------------------------------------------------
    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == 'pacbot/wheel_vel':
                d = json.loads(msg.payload.decode())
                self.cmd_l = max(-MAX_WHEEL, min(MAX_WHEEL, float(d['left'])))
                self.cmd_r = max(-MAX_WHEEL, min(MAX_WHEEL, float(d['right'])))
                self.last_cmd_t = time.perf_counter()
                self.cmd_evt.set()
            elif msg.topic == 'sim/reset':
                self.reset_req = True
        except Exception as e:                                  # noqa
            print('[sim] bad message on', msg.topic, e)

    # -- main loop --------------------------------------------------------------
    def run(self):
        a = self.args
        client = mqtt_client()
        client.on_message = self.on_message
        client.connect(a.host, a.port, 60)
        client.subscribe([('pacbot/wheel_vel', 0), ('sim/reset', 0)])
        client.loop_start()
        if a.autostart:
            client.publish('bot/cmd', '1', retain=True)
        self.publish_pellets(client)
        self.publish_pose(client, force=True)

        viewer = None
        if not a.headless:
            import mujoco.viewer as mj_viewer
            viewer = mj_viewer.launch_passive(self.model, self.data)
            if a.overview:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.lookat[:] = (MAZE_COLS * self.S / 2, MAZE_ROWS * self.S / 2, 0)
                viewer.cam.distance = MAZE_COLS * self.S * 1.35
                viewer.cam.azimuth, viewer.cam.elevation = 90, -80
            else:                                   # chase camera like the reference screenshot
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                viewer.cam.trackbodyid = self.body_id
                viewer.cam.distance = 0.55
                viewer.cam.azimuth, viewer.cam.elevation = 90, -50

        decim = max(1, int(round(1.0 / (a.sensor_hz * TIMESTEP))))
        sensor_dt = decim * TIMESTEP
        print(f'[sim] cell={self.S} m  objectives at {self.pellet_cells}  start={self.start}  '
              f'sensors @ {1 / sensor_dt:.0f} Hz  rtf={a.rtf if a.rtf > 0 else "max"}')
        print('[sim] waiting for controller on pacbot/wheel_vel ...')
        wall0, sim0 = time.perf_counter(), float(self.data.time)
        last_sync = last_slow = 0.0
        step = 0
        try:
            while True:
                now = time.perf_counter()
                if viewer is not None and not viewer.is_running():
                    break
                if self.reset_req:
                    self.reset_req = False
                    self.reset()
                    self.publish_pellets(client)
                    self.publish_pose(client, force=True)
                    wall0, sim0 = time.perf_counter(), float(self.data.time)
                    print('[sim] reset')
                if now - self.last_cmd_t > CMD_TIMEOUT:       # watchdog
                    self.data.ctrl[0] = self.data.ctrl[1] = 0.0
                else:
                    self.data.ctrl[0], self.data.ctrl[1] = self.cmd_l, self.cmd_r
                mujoco.mj_step(self.model, self.data)
                step += 1
                if step % decim == 0:
                    self.cmd_evt.clear()
                    client.publish('pacbot/sensors', json.dumps(self.sensor_msg(sensor_dt)))
                    if a.lockstep:
                        self.cmd_evt.wait(a.lockstep_timeout)
                if step % 5 == 0:
                    self.count_collisions()
                    self.update_game(client)
                    self.publish_pose(client)
                if now - last_slow > 0.25:
                    last_slow = now
                    self.publish_pellets(client)
                    self.publish_pose(client, force=True)
                    x, y, yaw = self.pose()
                    client.publish('sim/truth', json.dumps(
                        {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
                         't': round(float(self.data.time), 3)}))
                    client.publish('sim/status', json.dumps(
                        {'state': 'escaped' if self.escaped else 'running',
                         'pellets_left': len(self.pellets_left()),
                         'collisions': self.collisions, 't': round(float(self.data.time), 3)}))
                if viewer is not None and now - last_sync > 1 / 60:
                    last_sync = now
                    viewer.sync()
                if a.rtf > 0:
                    ahead = (float(self.data.time) - sim0) / a.rtf - (time.perf_counter() - wall0)
                    if ahead > 0.0005:
                        time.sleep(ahead)
                if a.max_time > 0 and self.data.time > a.max_time:
                    print('[sim] max-time reached')
                    break
        except KeyboardInterrupt:
            pass
        finally:
            client.loop_stop()
            client.disconnect()
            if viewer is not None:
                viewer.close()


def build_parser():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser(description='PacBot MuJoCo maze simulator')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=int, default=1883)
    p.add_argument('--assets', default=here, help='folder with chassis_sim.stl / roda_sim.stl')
    p.add_argument('--cell', type=float, default=0.22, help='maze cell pitch [m] (reference sim: 0.22)')
    p.add_argument('--start', default='6,6,0', help='"row,col,yaw_deg" of the axle centre (default centre cell, facing east)')
    p.add_argument('--pellets', default='', help='objective cells, e.g. "3,2 9,10" (row,col); default: random')
    p.add_argument('--seed', type=int, default=1, help='seed for random objective placement')
    p.add_argument('--sensor-hz', type=float, default=500.0)
    p.add_argument('--rtf', type=float, default=1.0, help='real-time factor, 0 = as fast as possible')
    p.add_argument('--lockstep', action='store_true',
                   help='after every sensor message wait for a wheel command (deterministic)')
    p.add_argument('--lockstep-timeout', type=float, default=0.05)
    p.add_argument('--noise', type=float, default=0.0, help='sensor noise scale (0 = none)')
    p.add_argument('--headless', action='store_true', help='no viewer window')
    p.add_argument('--overview', action='store_true', help='top-down camera instead of chase camera')
    p.add_argument('--autostart', action='store_true', help='publish a retained "1" on bot/cmd')
    p.add_argument('--max-time', type=float, default=0.0, help='stop after N simulated seconds')
    return p


def main():
    PacBotSim(build_parser().parse_args()).run()


if __name__ == '__main__':
    main()
