#!/usr/bin/env python3
"""
PacBot autonomous controller for Task 1A.
Uses BFS to navigate the maze grid, collect pellets, and exit.
"""

from collections import deque
import json
import time

import paho.mqtt.client as mqtt

MAZE_ROWS = 13
MAZE_COLS = 13
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
POSE_TOPIC = "robot/pose"

# wall bit per side, OR'd together
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
# Maze map -- the same data as WALLS above, drawn. row 0 = SOUTH (bottom),
# col 0 = WEST (left). The gaps in the top and bottom edges are EXIT_CELLS.
#
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#   row 12 |                 |              |     |
#          +  +  +--+  +  +  +  +  +--+--+  +  +  +
#   row 11 |  |  |     |  |     |        |        |
#          +  +  +  +--+  +--+  +  +  +--+--+  +  +
#   row 10 |           |     |     |  |        |  |
#          +  +  +--+--+--+  +--+--+  +  +  +  +  +
#   row  9 |  |           |        |        |     |
#          +--+  +  +  +  +--+  +  +--+--+  +--+--+
#   row  8 |     |  |  |        |                 |
#          +  +--+  +  +--+  +  +--+--+  +  +  +  +
#   row  7 |        |     |  |     |     |     |  |
#          +  +  +  +--+  +  +--+  +  +  +--+--+  +
#   row  6 |  |           |  |     |  |           |
#          +  +--+--+  +--+  +  +--+  +  +--+--+  +
#   row  5 |     |           |        |  |        |
#          +  +  +  +--+--+--+  +  +--+  +  +--+  +
#   row  4 |  |  |        |     |  |     |  |  |  |
#          +  +  +  +--+  +--+  +  +  +--+  +  +  +
#   row  3 |  |        |              |        |  |
#          +  +--+--+  +--+  +--+  +--+  +--+  +  +
#   row  2 |        |        |     |     |     |  |
#          +  +--+  +  +  +--+  +  +  +--+--+  +  +
#   row  1 |  |  |  |  |     |     |           |  |
#          +  +  +  +  +--+  +  +  +--+  +  +--+  +
#   row  0 |     |     |                 |        |
#          +--+--+--+--+--+--+  +--+--+--+--+--+--+
#            0  1  2  3  4  5  6  7  8  9 10 11 12   <- col

# the 2 known exits: (row, col, facing)
EXIT_CELLS = [
    (0, 6, 'south'),
    (MAZE_ROWS - 1, 6, 'north'),
]

BOT_CMD_TOPIC = "bot/cmd"
PELLETS_TOPIC = "pellets/pose"
CMD_VEL_TOPIC = "robot/cmd_vel"

# yaw -> dr, dc, wall bit. 0=EAST, 90=NORTH, 180=WEST, 270=SOUTH
HEADING_DELTA = {
    0.0:   (0, 1, WALL_E),
    90.0:  (1, 0, WALL_N),
    180.0: (0, -1, WALL_W),
    270.0: (-1, 0, WALL_S),
}


# ============================================================================
# YOUR ALGORITHM GOES HERE. Everything above and below is plumbing.
# ============================================================================
def choose_command(pacbot_cell, pacbot_yaw, pellets_remaining):
    """FRONT/LEFT/RIGHT/BACK to send now, or None. pacbot_cell=(row,col),
    pacbot_yaw one of HEADING_DELTA's keys, pellets_remaining=set of
    (row,col). Implement this -- see EXIT_CELLS above for the 2 exits."""
    facing_to_yaw = {
        'east': 0.0,
        'north': 90.0,
        'west': 180.0,
        'south': 270.0,
    }

    current_yaw = float(pacbot_yaw) % 360.0

    # 1. If all pellets are gathered and robot is on an exit cell, step out
    if not pellets_remaining:
        for ex_r, ex_c, ex_facing in EXIT_CELLS:
            if pacbot_cell == (ex_r, ex_c):
                target_yaw = facing_to_yaw[ex_facing]
                turn = (target_yaw - current_yaw) % 360.0
                if turn == 0.0:
                    return "FRONT"
                elif turn == 90.0:
                    return "LEFT"
                elif turn == 180.0:
                    return "BACK"
                elif turn == 270.0:
                    return "RIGHT"

    # 2. Determine target goals: remaining pellets or the exit cells
    if pellets_remaining:
        goals = set(pellets_remaining)
    else:
        goals = {(r, c) for r, c, _ in EXIT_CELLS}

    # 3. Breadth-First Search to find the shortest path to the closest target
    queue = deque([pacbot_cell])
    visited = {pacbot_cell: None}
    target_found = None

    while queue:
        curr = queue.popleft()
        if curr in goals:
            target_found = curr
            break

        cr, cc = curr
        wall_mask = WALLS[cr][cc]
        for yaw, (dr, dc, wall_bit) in HEADING_DELTA.items():
            if not (wall_mask & wall_bit):
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < MAZE_ROWS and 0 <= nc < MAZE_COLS:
                    nxt = (nr, nc)
                    if nxt not in visited:
                        visited[nxt] = curr
                        queue.append(nxt)

    if target_found is None:
        return None

    # Reconstruct the path backwards
    path = []
    curr = target_found
    while curr is not None:
        path.append(curr)
        curr = visited[curr]
    path.reverse()

    if len(path) < 2:
        return None

    # 4. Determine direction to the immediate next cell on the path
    next_cell = path[1]
    dr = next_cell[0] - pacbot_cell[0]
    dc = next_cell[1] - pacbot_cell[1]

    target_yaw = None
    for yaw, (hdr, hdc, _) in HEADING_DELTA.items():
        if (hdr, hdc) == (dr, dc):
            target_yaw = yaw
            break

    if target_yaw is None:
        return None

    # 5. Translate required heading into robot motion primitives
    turn = (target_yaw - current_yaw) % 360.0

    if turn == 0.0:
        return "FRONT"
    elif turn == 90.0:
        return "LEFT"
    elif turn == 180.0:
        return "BACK"
    elif turn == 270.0:
        return "RIGHT"

    return None
# ============================================================================


def parse_pellets(payload):
    return {tuple(cell) for cell in json.loads(payload)}


def main():
    state = {
        "running": False,
        "pellets": set(),
        "cell": (MAZE_ROWS // 2, MAZE_COLS // 2),
        "yaw": 0.0,
        "in_flight": False,
        "got_pose": False,
        "got_pellets": False,
    }

    # Supports both paho-mqtt v1.x and v2.x
    if hasattr(mqtt, "CallbackAPIVersion"):
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="Controller")
    else:
        client = mqtt.Client(client_id="Controller")

    def decide_and_send():
        if (not state["running"] or state["in_flight"]
                or not state["got_pose"] or not state["got_pellets"]):
            return
        print(f"[debug] pose={state['cell']} yaw={state['yaw']} pellets={state['pellets']}")
        cmd = choose_command(state["cell"], state["yaw"], set(state["pellets"]))
        if cmd is not None:
            state["in_flight"] = True
            client.publish(CMD_VEL_TOPIC, cmd)
            print(f"[controller] {state['cell']} yaw={state['yaw']} -> {cmd}, "
                  f"pellets_left={len(state['pellets'])}")

    def on_message(client, userdata, msg):
        try:
            if msg.topic == BOT_CMD_TOPIC:
                running = msg.payload.decode().startswith("1")
                was_running = state["running"]
                state["running"] = running
                if running and not was_running:
                    decide_and_send()   # kick off the reactive loop on Start
            elif msg.topic == PELLETS_TOPIC:
                state["pellets"] = parse_pellets(msg.payload.decode())
                state["got_pellets"] = True
                decide_and_send()
            elif msg.topic == POSE_TOPIC:
                data = json.loads(msg.payload.decode())
                state["cell"] = (int(data["col"]), int(data["row"]))   # wire is swapped
                state["got_pose"] = True
                state["yaw"] = float(data.get("yaw", 0.0))
                state["in_flight"] = False   # this pose is the ack for our last command
                decide_and_send()   # every pose/command-ack triggers the next step
        except Exception as e:
            print("[controller] mqtt parse error:", e)

    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe([(BOT_CMD_TOPIC, 0), (PELLETS_TOPIC, 0), (POSE_TOPIC, 0)])
    client.loop_start()

    print(f"[controller] ready; sending one '{CMD_VEL_TOPIC}' command at a time, "
          f"reacting to '{POSE_TOPIC}'/'{PELLETS_TOPIC}' feedback")

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()