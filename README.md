# PacBot MuJoCo maze simulator

Files: `pacbot_sim.py` (simulator), `task_combined.py` (Task 1A+1B controller), `chassis_sim.stl`, `roda_sim.stl`.

    pip install mujoco paho-mqtt numpy
    mosquitto                              # terminal 1
    python3 pacbot_sim.py --autostart      # terminal 2  (macOS: mjpython pacbot_sim.py ...)
    python3 task_combined.py               # terminal 3

Useful flags: `--pellets "3,2 9,10"` (objective cells row,col), `--seed N`, `--overview` (top-down camera),
`--headless --rtf 0 --lockstep` (fast, deterministic), `--noise 1`, `--cell 0.22`, `--start "6,6,0"`.
Reset: publish anything on `sim/reset`.

Taken from task_1b_launch: robot geometry (rear axle -33 mm, wheels +-39 mm, front caster), motors
(kv 0.05, +-30 rad/s, +-0.02 Nm), dt 0.002, 4 ToF sites, gyro/accel, cell pitch 0.22, colours
(red interior / dark border walls, green start bar, yellow exit bars).
Sensor JSON keys: fl/fr = tof_front_left/right (look sideways), sl/sr = tof_side_left/right (look ahead),
gyro, accel, dt (+ extras: enc = wheel rad/s, t).  Your task_1b.py mapping works unchanged.

Objectives: 2 gold pellets; capture = axle centre within 7 cm of the pellet centre (pellets/pose is updated).
Exit through the N or S gap; `pacbot/result` = {solved, collisions, time_sec} (solved only if both captured).
Grid pose follows task_1a.py (swapped row/col on the wire, yaw snapped to 0/90/180/270).
