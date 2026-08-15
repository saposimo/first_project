# first_project — Wheel odometry and TF error for a tracked UGV

ROS 2 package that reconstructs the odometry of an **AgileX Bunker Pro** tracked
robot from its onboard velocity feedback, publishes the resulting pose on TF, and
continuously measures how far that estimate has drifted from the GPS ground truth
recorded in the bag.

Coursework for *Robotics* — MSc Automation and Control Engineering, Politecnico di
Milano.

> The GitHub repository is named `Robotics_first_project`; the ROS package inside
> it is `first_project`, as required by the assignment.

## What the package does

Dead reckoning has no absolute reference: every integration step adds a small error
that never gets corrected, so the estimated pose slowly walks away from reality.
The point of this project is to build that estimate and then quantify the drift
against a trusted source.

Two nodes:

| Node       | Subscribes            | Publishes                                   |
| ---------- | --------------------- | ------------------------------------------- |
| `odometer` | `/bunker_status`      | `/project_odom`, TF `odom → base_link2`     |
| `tf_error` | TF tree               | `/tf_error_msg`                             |

`odometer` also exposes a `reset` service (`std_srvs/srv/Empty`).

### Odometry model

The Bunker Pro is a **skid-steer** platform: it turns by driving its two tracks at
different speeds. Its controller already fuses the track encoders and reports body
velocities, so the node integrates the unicycle model rather than individual wheel
ticks:

$$
\theta_{mid} = \theta_k + \tfrac{1}{2}\omega\,\Delta t
$$

$$
x_{k+1} = x_k + v\cos(\theta_{mid})\,\Delta t
\qquad
y_{k+1} = y_k + v\sin(\theta_{mid})\,\Delta t
\qquad
\theta_{k+1} = \theta_k + \omega\,\Delta t
$$

Evaluating the heading at the **midpoint** of the interval instead of at its start
(explicit Euler) is a second-order Runge–Kutta step. It costs nothing extra and
noticeably reduces the drift that accumulates during turns, which is where explicit
Euler consistently cuts corners.

Two details that matter when replaying the provided bag:

- **Empty timestamps.** `/bunker_status` ships with an unset `header.stamp`. The
  node falls back to `now()`, which under `use_sim_time:=true` follows `/clock` and
  therefore stays aligned with bag playback.
- **Samples with a bad `Δt`.** Non-increasing or implausibly large steps (> 1 s,
  typical of a bag loop restart) are skipped instead of being integrated, which
  would otherwise teleport the pose.

The pose is initialised from the bag ground truth (`odom → base_link`) so that both
trajectories start superimposed and the error plot measures drift alone, not a
constant offset.

### Error metric

`tf_error` samples both transforms at a common timestamp and publishes:

```
std_msgs/Header header
float32 tf_error            # Euclidean XY distance between ground truth and estimate [m]
int32   time_from_start     # seconds since the first valid comparison
float32 travelled_distance  # path length integrated along the estimated pose [m]
```

Reporting `travelled_distance` alongside the error is what makes the number
meaningful: 0.5 m of drift after 5 m is poor, after 500 m it is excellent.

## Requirements

- ROS 2 Humble
- `bunker_msgs` in the same workspace (provides `bunker_msgs/msg/BunkerStatus`)
- The course rosbag, which is not distributed here

## Build

From the root of your colcon workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to first_project
source install/setup.bash
```

`--packages-up-to` builds `bunker_msgs` first, since the custom message is a build
dependency.

## Run

```bash
ros2 launch first_project first_project.launch.py
```

That single command starts both nodes and RViz with the bundled configuration
(`rviz/first_project.rviz`), which shows the two frames in top view.

| Argument       | Default | Purpose                                         |
| -------------- | ------- | ----------------------------------------------- |
| `use_rviz`     | `true`  | Set to `false` to run the nodes headless        |
| `use_sim_time` | `true`  | Keep `true` while replaying a bag               |

In a second terminal, play the bag:

```bash
ros2 bag play --clock 20 --rate 0.5 --loop <path/to/rosbag>
```

`--clock` publishes simulated time — without it the nodes have no time source and
nothing is integrated. `--rate 0.5` gives the TF buffer more slack on a busy
machine.

### Inspecting the result

```bash
ros2 topic echo /project_odom      # estimated pose
ros2 topic echo /tf_error_msg      # drift, elapsed time, distance travelled
ros2 run tf2_ros tf2_echo odom base_link    # ground truth
ros2 run tf2_ros tf2_echo odom base_link2   # estimate
ros2 service call /reset std_srvs/srv/Empty {}
```

## Layout

```
src/odometer.cpp      odometry integration, TF broadcast, reset service
src/tf_error.cpp      drift measurement against ground truth
msg/TfErrorMsg.msg    custom message
launch/               single entry point, starts both nodes and RViz
rviz/                 top-view configuration showing both frames
```

## Known limitations

- **`reset` does not survive the next sample.** The service zeroes the pose, but it
  also clears the initialisation flag, so the following `/bunker_status` message
  re-seeds the pose from the bag ground truth. Zeroing the pose and keeping it
  zeroed requires separating the two flags.
- **`tf_error` samples on a wall-clock timer.** `create_wall_timer` ignores
  `use_sim_time`, so replaying at `--rate 0.5` halves the effective sampling rate in
  simulated time. It does not affect correctness of the published values, only how
  densely they are produced.
- Skid-steer odometry is intrinsically optimistic: track slip during turns is not
  observable from velocity feedback alone, and it is the dominant error source here.

## License

MIT — see [LICENSE](LICENSE).
