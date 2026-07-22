# Unitree Go2 (ROS 2 Jazzy) — Foot Contact & Force Sensing

> ### ⚠️ This is a fork — credit where it's due
> The Unitree Go2 simulation, the [CHAMP](https://github.com/chvmp/champ)
> controller integration, and everything that makes the robot stand and walk are
> the work of the **original authors**:
> **[RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy)**.
>
> **The only thing this fork adds is per-foot contact & force sensing.**
> Nothing else here is claimed as our own work. The complete original README is
> preserved as **[`README_UPSTREAM.md`](README_UPSTREAM.md)**, and the full commit
> history of the original authors is kept intact in this repository.

---

## What this fork adds

The Go2 model has no physical foot-contact sensors, so per-foot **contact
detection** and **ground-reaction force** are synthesized in Gazebo and published
to ROS 2 on a single topic, **`/foot_states`**.

`champ_msgs/msg/FootStates` (all arrays length 4, order `lf, rf, lh, rh`):

| field | meaning |
|-------|---------|
| `names[]` | foot names — `['lf', 'rf', 'lh', 'rh']` |
| `in_contact[]` | `true`/`false` — a foot is in contact when its force magnitude **> 0.1 N** |
| `force[]` | world-frame ground-reaction force `x / y / z` per foot, in newtons |
| `normal_force[]` | `\|z\|` convenience value |

The raw per-foot Gazebo contact (full wrench) is also bridged to
`/foot_contact/{lf,rf,lh,rh}` (`ros_gz_interfaces/msg/Contacts`).



## Requirements

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Sim Harmonic.
See **[`README_UPSTREAM.md`](README_UPSTREAM.md)** for the full apt/`rosdep`
dependency setup from the original package.

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Run

```bash
# Terminal 1 — launch the simulation (the robot stands within ~12 s)
ros2 launch unitree_go2_sim unitree_go2_launch.py

# Terminal 2 — watch the foot contacts + forces
source ~/ros2_ws/install/setup.bash
ros2 topic echo /foot_states

# Terminal 3 — drive the robot (keep the speed modest: <= 0.3 m/s)
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.4
```

Notes:
- Every new terminal must `source ~/ros2_ws/install/setup.bash` first.
- CHAMP holds the last velocity command — press a stop key (or send zero) to halt.

---

## Credit & license

This repository is a **fork** of
**[RobInLabUJI/unitree_go2_ros2_jazzy](https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy)**
and, through it, builds on:

- **[chvmp/champ](https://github.com/chvmp/champ)** — the CHAMP quadruped controller framework.
- **[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros)** — the Go2 robot description.

All of the base package is the work of those authors and is **not** claimed here.
This fork's contribution is limited strictly to the foot contact/force sensing
described above. Please refer to the original repository for the licensing terms
of the base package and respect them.
