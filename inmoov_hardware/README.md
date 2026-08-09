# inmoov_hardware

A ros2_control `SystemInterface` that drives the **real InMoov** by translating URDF joint
positions into MyRobotLab servo commands. Moving the robot in RViz moves the robot.

Everything runs on the ROS2 machine (skynet). **Nothing is installed on pi5-inmoov.** MRL's
Nixie API is plain HTTP — `GET /api/service/{service}/{method}/{jsonArg}` — so the driver
just speaks to `pi5-inmoov.local:8888` over the LAN with libcurl. No WebSocket, no bridge
process, no pi5-side service to keep in sync.

---

## ⚠ The one rule: exactly one thing drives MRL at a time

`inmoov-driver.service` (Burf Platform) and `inmoov-quest-bridge.service` (Quest teleop) on
pi5-inmoov both push servo positions to the same MRL instance. So does this driver. Two of
them at once means two controllers fighting over the same servos.

The pi5 units already declare `Conflicts=` against each other, but **this driver runs on a
different machine, so systemd cannot police it.** It is on you:

```bash
# before
ssh burf2000@pi5-inmoov.local 'sudo systemctl stop inmoov-driver inmoov-quest-bridge'

# ... run the ROS2 driver ...

# after — inmoov-driver is normally ACTIVE and enabled, put it back
ssh burf2000@pi5-inmoov.local 'sudo systemctl start inmoov-driver'
```

Leave `myrobotlab.service` running throughout — that *is* MRL, and this driver needs it.

---

## The mapping model

URDF joints are radians with URDF limits. MRL servos are degrees, 0–180, with a calibrated
rest position that differs a lot left vs right (left shoulder rests at 35, right at 62).
The bridge between them is `config/inmoov_joint_map.yaml`.

Every joint is anchored at **`urdf_home_rad`, which maps exactly onto `servo_rest`**. From
there, travel is scaled independently on each side so the URDF joint's usable range lands
on the servo's real `[servo_min, servo_max]`:

```
d = (q - urdf_home_rad)          , negated if invert: true

d >= 0  ->  deg = rest + (d / pos_span_rad) * (servo_max - rest)
d <  0  ->  deg = rest + (d / neg_span_rad) * (rest - servo_min)

invert: false  ->  pos_span_rad = urdf_max - home ,  neg_span_rad = home - urdf_min
invert: true   ->  pos_span_rad = home - urdf_min ,  neg_span_rad = urdf_max - home
```

The per-side scaling matters. Without it a servo with narrow travel just floors at its
minimum and barely moves — the exact failure `inmoov_quest_bridge.py` hit with the omoplate,
which is why its `_remap()` exists.

### centred vs floored

`mode` is documentation plus a load-time assertion, not a second code path:

| mode | meaning | examples |
|---|---|---|
| `centred` | `urdf_home_rad` is **inside** the URDF range and `servo_rest` is mid-travel. The joint swings both ways from rest. | head pan, arm rotate, wrist roll, waist |
| `floored` | `urdf_home_rad` sits **at one end** of the URDF range and `servo_rest` is the neutral bottom of servo travel. Only one direction of travel exists. | jaw, fingers, omoplate, bicep |

Three cases are worth knowing about because they look wrong until you see why:

- **Elbows (`r/l_elbow_flex_joint`)** — the URDF range never contains 0 (`-1.396 … -0.262`),
  and the servo's rest **is** its minimum (bicep rest 5, min 5). So home is the *upper* URDF
  limit (straightest arm) and `invert: true` sends increasing flex to increasing degrees.
- **Wrist rolls** — the URDF range is one-sided (`-π … 0`) but servo rest 90 is mid-travel,
  so `urdf_home_rad` is the URDF **midpoint**, not 0. Otherwise half the servo is wasted.
- **Shoulder lifts** — labelled `centred` even though they mostly travel one way. Home 0 is
  strictly inside the range and there really is a little below-rest travel (62→50 on the
  right, 35→30 on the left). It is just very lopsided. Calling it `floored` would be a lie
  the loader now rejects.

### Where the numbers came from

- `urdf_min_rad` / `urdf_max_rad` — from `xacro robots/inmoov.urdf.xacro`, not guessed.
- `servo_min` / `servo_max` / `servo_rest` — read **live from MRL** (`getMin`/`getMax`/
  `getRest`) on 2026-07-28, the same way the Quest bridge does it. The static table in
  `servo_map.py` on the pi5 says 0–180 for everything and is not the calibrated truth.

The YAML values are the **fallback and the offline-test defaults**. With
`refresh_limits: true` (the default) the driver re-reads them from MRL on activation, so
recalibrating a servo in MRL is picked up without editing anything here.

### Safety

Output is clamped twice: first into `[servo_min, servo_max]`, then into the optional
`[safety_min, safety_max]` cap. `test_joint_map.cpp` sweeps 10 radians either side of every
URDF limit and asserts nothing ever escapes that box.

---

## What is enabled, what is parked

The URDF has 53 movable joints. **28 have a servo behind them**; the other 25 are marked
`enabled: false` with a reason and are ignored cleanly, never an error:

- **Finger sub-joints** (`r_index1_joint`, `r_index3_joint`, …) — the URDF models each
  finger as 3–4 chained revolute joints, but the real hand has **one tendon servo per
  finger**. Only the plain `_joint` of each finger is driven.
- **`l_eye_pan_joint`** — both eyeballs are driven by the single `i01.head.eyeX` servo.

Three MRL servos have **no URDF joint** and are therefore never driven: `eyelidLeft`,
`eyelidRight`, `topStom` (the URDF has waist pan and roll but no waist pitch).

### Bring-up groups — `enabled_groups` defaults to `head`

Being in the mapping is not the same as being driven. `enabled_groups` is the gate, and it
defaults to **`head` only**. A joint outside the enabled groups has its command accepted and
then dropped, so that part of the robot is left exactly where it is and is never even nudged
to rest. Same staged-bring-up idea as the Quest bridge's `DISABLED_GROUPS`.

Groups: `head` `eyes` `rightArm` `leftArm` `rightHand` `leftHand` `torso`.

Widen only once you have watched that group move. **`torso` is high risk** and its
servo-role assignment (`lowStom` = waist rotation, `midStom` = side lean) is an assumption
about InMoov convention that nobody has checked on this robot.

---

## Launching

### The slider path (what Simon asked for)

joint_state_publisher_gui sliders publish straight to `/joint_states`. No controller ever
sees them, so ros2_control is not involved. `joint_state_relay` subscribes to
`/joint_states` and drives MRL through **the same `JointMap` code the plugin uses** — the
mapping arithmetic exists exactly once, in `libinmoov_mapping`.

```bash
# look only, nothing moves
ros2 launch inmoov_bringup inmoov_sliders.launch.py

# prove the pipeline first: every slider logged as servo degrees, nothing sent
ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true dry_run:=true

# for real
ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true
ros2 launch inmoov_bringup inmoov_sliders.launch.py use_real_hardware:=true enabled_groups:=head,eyes
```

### The ros2_control path

```bash
ros2 launch inmoov_bringup inmoov_real.launch.py dry_run:=true      # prove it
ros2 launch inmoov_bringup inmoov_real.launch.py                    # for real

# command the 28 joints, in the order listed in inmoov_controllers.yaml
ros2 topic pub -1 /inmoov_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

`use_real_hardware:=true` also means **no Gazebo** — when the real robot is the plant there
is nothing to simulate. The switch is in `inmoov_description/robots/inmoov.urdf.xacro`; both
xacro files only define macros, so `inmoov_description` gains no dependency on this package.

`inmoov_description/launch/display.launch.py` and `gazebo.launch.py` are untouched and
behave exactly as before.

## The dry-run flag

`dry_run:=true` logs every servo move the driver *would* make, in degrees, and sends
**nothing** to MRL. It also skips the MRL reachability check, so it works with the robot off.
Always prove a new mapping or a new machine here before anything moves.

```
[DRY RUN] would send 4 servo(s): i01.head.jaw=51.0 i01.head.neck=115.0 i01.head.rollNeck=90.0 i01.head.rothead=90.0
```

## Parameters

Set as `<param>` in the ros2_control xacro block, or as ROS parameters on `joint_state_relay`.

| parameter | default | what it does |
|---|---|---|
| `mrl_host` / `mrl_port` | `pi5-inmoov.local` / `8888` | where MRL is |
| `enabled_groups` | `head` | comma-separated groups actually driven |
| `dry_run` | `false` | log intended moves, send nothing |
| `joint_map` | *(empty)* | path override; empty = the shipped YAML |
| `refresh_limits` | `true` | re-read min/max/rest from MRL on activation |
| `hold_refresh_s` | `0.5` | anti-droop: re-assert a held pose this often |
| `move_epsilon_deg` | `1.0` | don't resend a servo until it moves this far |
| `require_mrl` | `true` | refuse to activate if MRL is unreachable |

### Droop

InMoov servos sag if they are not re-commanded — the Quest bridge exists partly because the
head sank. So an unchanged pose is still re-sent every `hold_refresh_s`. `write()` hands the
pose to a background thread; the HTTP never blocks the control loop, and the whole pose goes
out concurrently on keep-alive connections via `curl_multi` rather than one round trip per
servo.

### `read()` is open loop, and says so

MRL gives **no true position feedback**. InMoov servos are open-loop hobby servos with no
encoder, and MRL's `getPos` just echoes its own last commanded target. Polling it every
cycle would cost one HTTP round trip per servo to learn nothing. So `read()` reports the
last **commanded** position. The `/joint_states` this driver produces are a *model* of the
robot, not a *measurement* of it.

(`getPos` is still used once, at activation, to seed the state interface so RViz starts
roughly where the robot is instead of snapping it to some other pose.)

---

## What is unverified

Be clear about the difference between *wired* and *verified live*.

**Verified live** (2026-07-28, on the real robot, `inmoov-driver.service` stopped):
`head_pan_joint` only. Commanded `0 → +0.20 → 0 → -0.20 → 0` rad through the relay with a
±8° cap; MRL reported rothead `90 → 97 → 90 → 83 → 90` and it returned to rest on shutdown.
That proves the whole skynet → MRL → servo path.

**Not verified, and it needs somebody watching the robot:**

- **Physical motion.** `getPos` echoes MRL's target, so the above proves MRL accepted and
  applied the command — not that the head actually turned. There is no camera on the pi5
  usable for independent confirmation.
- **Direction (`invert`) on every single joint, head included.** Every sign in the YAML is a
  geometric best-guess from the URDF axis; none has been checked by eye. If a joint goes the
  wrong way, flip `invert` for that joint — nothing to rebuild but the package.
- **Everything outside the head group.** Arms, hands and torso are wired and arithmetically
  checked, and have never been commanded on the robot. `enabled_groups` keeps them off by
  default for exactly that reason.
- **The torso servo roles.** `lowStom` = waist rotation and `midStom` = side lean is an
  assumption. Check before enabling `torso`.

Suggested order for widening, one group at a time, in `dry_run` first, then live with a
hand near the power: `head` → `eyes` → `rightHand` / `leftHand` → `rightArm` / `leftArm` →
`torso`.

---

## Layout

```
hardware/                        the ros2_control plugin + shared mapping/transport
  inmoov_system.cpp              the SystemInterface itself
  joint_map.cpp                  radians <-> degrees. No ROS, no I/O, unit-testable
  mrl_client.cpp                 HTTP to MRL, background writer, droop refresh
include/inmoov_hardware/         headers
src/joint_state_relay_node.cpp   the /joint_states -> MRL relay (slider path)
config/inmoov_joint_map.yaml     THE mapping table
test/test_joint_map.cpp          offline arithmetic gate — 19 tests
inmoov_hardware.xml              pluginlib export
```

## Tests

```bash
colcon test --packages-select inmoov_hardware && colcon test-result --all --verbose
```

They run against the **shipped** YAML and assert: home maps exactly onto rest for every
joint; both URDF limits land inside the servo range; each joint actually reaches a servo
extreme (no wasted travel); direction follows the `invert` flag; nothing escapes the servo
box across a 10 rad sweep either side; degrees round-trip back to radians idempotently; two
joints can never share a servo; a bad rest or a duplicate service is rejected at load. Plus
hand-checked numbers for head pan, the jaw and the right elbow.

Nothing goes near the robot until these pass.
