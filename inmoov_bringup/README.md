# inmoov_bringup

**How to run the InMoov.** Four launch files, one job each. Pick the line you want.

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros-ws/install/setup.bash
```

| I want to… | command |
|---|---|
| Drive the **real robot** with RViz sliders | `ros2 launch inmoov_bringup real.launch.py` |
| Same, but **prove it first** — nothing moves | `ros2 launch inmoov_bringup real.launch.py dry_run:=true` |
| Drive a **simulated** robot in Gazebo | `ros2 launch inmoov_bringup sim.launch.py` |
| Just **look at the model** — no robot, no sim | `ros2 launch inmoov_bringup view.launch.py` |
| Real robot via **ros2_control** (for MoveIt / trajectories / other nodes) | `ros2 launch inmoov_bringup controller.launch.py` |

Every one of them gives you RViz and the joint_state_publisher_gui sliders. The only
difference is what is on the other end of the sliders.

---

## Before you touch the real robot

> **Exactly one thing may drive MyRobotLab at a time.** The Platform driver, the Quest
> bridge and this driver all command the same servos.

```bash
ssh burf2000@pi5-inmoov.local 'sudo systemctl stop inmoov-driver'    # hand MRL over
# ... run whatever you are running ...
ssh burf2000@pi5-inmoov.local 'sudo systemctl start inmoov-driver'   # give it back
```

`inmoov-driver.service` is normally **active and enabled**, so put it back when you finish.
`systemd` cannot enforce this across two machines — a leftover node on skynet will happily
keep driving MRL after you have restarted the driver on the pi. Check by hand if in doubt:

```bash
pgrep -af "joint_state_relay|ros2 launch"        # on skynet — should be empty when done
```

## The servos are not held

**Nothing re-asserts a pose that has not changed.** A servo is commanded only when its
joint actually moves, so a raised arm will slowly sag instead of sitting under continuous
holding torque.

That is deliberate. These are hobby servos with no thermal protection; holding a pose keeps
all 28 energised indefinitely. A sagging limb is recoverable, a cooked servo is a rebuild.

If you need a pose held for a short demo, turn the refresh back on for that run only:

```bash
ros2 launch inmoov_bringup real.launch.py hold_refresh_s:=0.5
```

Sub-degree slider nudges are also ignored (`move_epsilon_deg`, 1.0° by default). If a joint
seems unresponsive to tiny movements, that is why.

## Staging a bring-up

All six groups are driven by default: `head, rightArm, leftArm, rightHand, leftHand, torso`.
Narrow it when you are checking something new:

```bash
ros2 launch inmoov_bringup real.launch.py enabled_groups:=head,rightHand
```

A joint whose group is not listed is left exactly where it is and never even nudged to rest.

**Directions are not all confirmed.** Each joint has a `verified` field in
`inmoov_hardware/config/inmoov_joint_map.yaml`: `live` means someone watched it move,
`none` means the direction is still a geometric guess from the URDF axis. Nine are `live`.
If a joint moves the wrong way, flip its `invert` and rebuild — see
`inmoov_hardware/README.md`.

## Useful arguments

Common to `real.launch.py` and `controller.launch.py`:

| argument | default | what it does |
|---|---|---|
| `dry_run` | `false` | Log every servo move that *would* be sent; send nothing. |
| `enabled_groups` | all six | Which groups are actually driven. |
| `hold_refresh_s` | `0.0` | Seconds between re-asserting an unchanged pose. `0` = never. |
| `mrl_host` / `mrl_port` | `pi5-inmoov.local` / `8888` | Where MyRobotLab is. |
| `rviz` | `true` | Set `false` to skip RViz. |

## What each file actually starts

**`view.launch.py`** — `robot_state_publisher` + sliders + RViz. The URDF is loaded with
`use_real_hardware:=false`, so it carries no `<ros2_control>` block at all.

**`sim.launch.py`** — wraps `inmoov_description/launch/gazebo.launch.py` with the sliders on.
Gazebo starts its own RViz, so this file does not add one.

**`real.launch.py`** — sliders publish `/joint_states`; the `joint_state_relay` node converts
each joint to servo degrees around that servo's calibrated **rest** and sends it to MRL.
There is no `controller_manager` here, which is why the URDF has no `<ros2_control>` block.

**`controller.launch.py`** — `controller_manager` with the `inmoov_hardware/InmoovSystem`
plugin, plus `joint_state_broadcaster` and `inmoov_position_controller`. **No Gazebo** — when
the real robot is the plant there is nothing to simulate. Here `/joint_states` belongs to the
broadcaster (the robot saying where it *is*), so the sliders are remapped to
`slider_joint_states` and the `slider_to_controller` bridge forwards them to the command
topic. Anything else can command it the standard way:

```bash
ros2 topic pub /inmoov_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, ...]}"   # one value per joint, in inmoov_controllers.yaml order
```

## config/inmoov_controllers.yaml

Separate from `inmoov_description/config/controllers.yaml`, which is tuned for Gazebo at
1 kHz. Real hardware runs at **50 Hz** — every command becomes an HTTP request to MRL and
then a pulse down a serial servo bus, so anything faster is wasted.

The controller is `forward_command_controller/ForwardCommandController` with
`interface_name: position`, which is exactly what
`position_controllers/JointGroupPositionController` wraps. It is used directly because
`ros-jazzy-ros2-controllers` is **not** installed on skynet and `forward_command_controller`
is. If you install it, swapping the type back works identically — drop the `interface_name`
line when you do.

The 28 joints listed are the ones with a servo behind them, matching the `<ros2_control>`
block in `inmoov_description/urdf/inmoov.ros2_control.xacro`. **Listing a joint here does not
mean it moves** — `enabled_groups` on the hardware interface is the gate.
