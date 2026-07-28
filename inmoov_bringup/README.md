# inmoov_bringup

Launch files and controller config for **running** the InMoov. No C++, no robot model — it
just wires `inmoov_description` (the model) to `inmoov_hardware` (the driver).

> ⚠ **Exactly one thing may drive MyRobotLab at a time.** Stop `inmoov-driver.service` and
> `inmoov-quest-bridge.service` on pi5-inmoov before either launch file below, and start
> `inmoov-driver.service` again afterwards — it is normally active and enabled. Full
> explanation in `inmoov_hardware/README.md`.

## The two paths

**`inmoov_sliders.launch.py`** — RViz + `joint_state_publisher_gui`. Dragging a slider moves
the real robot. The sliders publish straight to `/joint_states`, which no controller sees, so
this path deliberately loads the URDF *without* any `<ros2_control>` block and runs the
`joint_state_relay` node instead.

```bash
ros2 launch inmoov_bringup inmoov_sliders.launch.py                                     # look only
ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true dry_run:=true
ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true
```

**`inmoov_real.launch.py`** — controller_manager + `joint_state_broadcaster` +
`inmoov_position_controller`, with the `inmoov_hardware/InmoovSystem` plugin. **No Gazebo.**

```bash
ros2 launch inmoov_bringup inmoov_real.launch.py dry_run:=true
ros2 launch inmoov_bringup inmoov_real.launch.py
```

Common arguments: `dry_run` (default `false`), `enabled_groups` (default `head`),
`mrl_host` / `mrl_port`, `rviz`.

## config/inmoov_controllers.yaml

Separate from `inmoov_description/config/controllers.yaml`, which is tuned for Gazebo at
1 kHz. Real hardware runs at **50 Hz** — every command becomes an HTTP request to MRL and
then a pulse down a serial servo bus, so anything faster is wasted.

The controller is `forward_command_controller/ForwardCommandController` with
`interface_name: position`, which is exactly what `position_controllers/JointGroupPositionController`
wraps. It is used directly because `ros-jazzy-ros2-controllers` is **not** installed on
skynet and `forward_command_controller` is. If you ever install it, swapping the type back
works identically — drop the `interface_name` line when you do.

The 28 joints listed are the ones with a servo behind them, matching the `<ros2_control>`
block in `inmoov_description/urdf/inmoov.ros2_control.xacro`. **Listing a joint here does not
mean it moves** — `enabled_groups` on the hardware interface is the gate, and it defaults to
`head`.
