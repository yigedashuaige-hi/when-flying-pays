# When Does Flying Pay Off?

Code and compact data for **“When Does Flying Pay Off? Ground-Failure Models and Mode-Switching Regimes for Terrestrial–Aerial Robots.”**

Hongxin Wang, Qiaoling Liu, and Kaibo Wang — Hunan University  
Correspondence: hongxinw918@gmail.com

## What is included

- `ros_ws/src/rescue_mission/` — mission logic, switching policies, logging, configurations, analysis, and tests.
- `ros_ws/src/rescue_worlds/` — Gazebo worlds, RotorS integration, launch files, and validation utilities.
- `ros_ws/src/uav_v4/` — robot description, controller parameters, plugins, and meshes.
- `data/stage0/` — Stage 0 summary tables and statistical outputs.
- `data/f1/` — preregistration, candidate definitions, held-out run rows, and paired statistics.
- `data/f1_censoring_replay/` — the four-seed stopping-sensitivity replay completed before the public-package cutoff.
- `data/d2/` — RotorS shaft-mechanical proxy calibration and supervisor results.
- `data/rotors_r2/` — bag-derived Scene A trajectory, event, and mission summaries; large ROS bags are omitted.
- `analysis/` — standalone scripts for reproducing the compact statistics and figures.
- `docs/` — experiment protocols, engineering reports, and evidence boundaries.

The package contains only material acquired no later than **2026-08-29 17:00 Asia/Shanghai**. Later exploratory experiments are excluded. The manuscript itself is maintained separately and is not included here.

## Reproduce the reported summaries

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
make statistics
make figures
```

Outputs are written under `reproduced/`. These commands only read archived data; they do not launch ROS, Gazebo, RotorS, or new experiments.

The same workflow can be run in a container:

```bash
docker build -t when-flying-pays .
docker run --rm -v "$PWD/reproduced:/workspace/reproduced" when-flying-pays
```

## ROS environment

The implementation targets ROS 1 Noetic and Gazebo Classic. Place the packages under `ros_ws/src/` in a Catkin workspace and install RotorS for the dynamic validation path. Exact launch/configuration details are recorded in `docs/`.

## Metric boundaries

- Stage 0 task effort is a unitless distance/hover proxy, not physical energy.
- D2 reports a modeled shaft-mechanical proxy, not battery energy.
- F1 effort comparisons use paired missions completed by both methods; completion retains every paired seed.
