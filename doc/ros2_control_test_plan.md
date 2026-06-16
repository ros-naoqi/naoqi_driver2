# ros2_control Test Plan — NAO / Pepper

This plan validates the `ros2_control` hardware interfaces (`AlMotionSystem`,
`DcmSystem`, `LolaSystem`). It is layered: Level 1 runs in CI with no robot,
Level 2 is a manual emulation smoke test, and Level 3 is the on-robot procedure
a human runs to confirm motion and to resolve the open assumptions in
[`../ASSUMPTIONS.md`](../ASSUMPTIONS.md).

> Safety: on a real robot, always clear space around it, disable autonomous
> life, and keep the emergency stop / chest button reachable. Joints become stiff
> when a controller activates.

## Backend / robot coverage

| Backend | NAO 2.1 | NAO 2.8+ | Pepper 2.5 | Pepper 2.9 | Runs |
|---------|:-------:|:--------:|:----------:|:----------:|------|
| `AlMotionSystem` | ✓ | ✓ | ✓ | ✓ (required) | remote or on-robot |
| `DcmSystem` | ✓ | — (DCM removed) | ✓ | — | remote or on-robot |
| `LolaSystem` | — | ✓ | — | ✓ (hidden) | on-robot only |

A "✓" is a configuration we intend to support; whether it works is exactly what
Level 3 confirms.

## Prerequisites

- A built, sourced workspace (`source install/setup.bash`).
- For Level 3: the robot's IP, and for NAOqi ≥ 2.9 a password. Network reachable.
- Recommended robot prep:
  ```sh
  ssh nao@<robot_host>
  qicli call ALAutonomousLife.setState disabled
  qicli call ALMotion.wakeUp
  ```

---

## Level 1 — Automated emulation tests (CI, no robot)

These run on every push (`colcon test`) and gate the PR. Each drives a hardware
interface through its full lifecycle against an in-process fake and asserts a
commanded position is read back.

| Test | Exercises |
|------|-----------|
| `naoqi_driver_hardware_test` | `AlMotionSystem` via fake ALMotion + ALMemory |
| `naoqi_driver_dcm_test` | `DcmSystem` via fake DCM (alias flow) + ALMemory |
| `naoqi_driver_lola_test` | `LolaSystem` via a fake `/tmp/robocup` server + the MessagePack codec |

Run locally:

```sh
colcon test --packages-select naoqi_driver \
  --ctest-args -R "hardware_test|dcm_test|lola_test"
colcon test-result --verbose
```

**Pass:** all three tests report `HeadYaw` reaching the commanded `0.5 rad`, and
the LoLA codec round-trip matches to `1e-6`.

**Limit:** these prove the *plumbing and our own codec are self-consistent*; they
do **not** prove wire-compatibility with a real robot. That is Level 3.

---

## Level 2 — Emulation integration (manual, no robot)

Confirms the launch, URDF/xacro, controllers and joint mapping wire up together.

```sh
# NAO (default)
ros2 launch naoqi_driver ros2_control.launch.py emulation_mode:=true
# Pepper
ros2 launch naoqi_driver ros2_control.launch.py emulation_mode:=true robot:=pepper
```

In another shell:

```sh
ros2 control list_hardware_interfaces      # position command + pos/vel/effort state per joint
ros2 control list_controllers              # broadcaster + <robot>_joint_trajectory_controller : active
ros2 topic hz /joint_states                # ~update_rate (50 Hz)
ros2 action send_goal /nao_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [HeadYaw], points: [{positions: [0.3], time_from_start: {sec: 2}}]}}"
ros2 topic echo /joint_states --once       # HeadYaw position tracks toward 0.3
```

**Pass:** controllers reach `active`, `/joint_states` publishes all joints at the
configured rate, and a trajectory goal succeeds with the state tracking the
target. Repeat with `robot:=pepper` (17 joints, no wheels).

---

## Level 3 — On-robot tests (human verification)

The quickest end-to-end check, for any backend:

```sh
./src/naoqi_driver/test/real_robot_move.sh <robot_ip> [password] <plugin> <nao|pepper>
```

It brings up the stack, waits for the controller to activate, and commands a
small head nod (`HeadYaw` → +0.3 → 0). **Observe the head physically move.**

The per-backend procedures below go further and capture the data needed to close
the open assumptions.

### 3.0 General checklist (every backend)

1. Robot prepped (autonomous life disabled, `wakeUp`), space cleared.
2. Launch with the right `robot:=`, `plugin:=`, and connection args.
3. `ros2 control list_controllers` → both controllers `active`.
4. `ros2 topic echo /joint_states` shows **plausible, live** angles (matches the
   robot's actual pose; move a limb by hand with low stiffness and watch it
   change).
5. Send a small head trajectory; confirm motion matches the command direction
   and magnitude (no inversion, no scaling error).
6. On `Ctrl-C`, the robot does not jerk.

### 3.1 ALMotion (`naoqi_driver/AlMotionSystem`) — any robot

Baseline; should "just work" over the network.

```sh
ros2 launch naoqi_driver ros2_control.launch.py \
  robot:=<nao|pepper> plugin:=naoqi_driver/AlMotionSystem \
  nao_ip:=<robot_host> password:=<password>
```

- Run 3.0.
- Sweep one joint slowly across part of its range; verify smoothness and that
  `/joint_states` tracks within a small lag.
- **Record:** does `setStiffnesses` on activation hold the limb against gravity?
  Any joints that refuse to move (e.g. hands)?

### 3.2 DCM (`naoqi_driver/DcmSystem`) — NAO 2.1 / Pepper 2.5

Low-level position + stiffness. Validates the `createAlias`/`setAlias` flow over
libqi.

```sh
ros2 launch naoqi_driver ros2_control.launch.py \
  robot:=<nao|pepper> plugin:=naoqi_driver/DcmSystem \
  nao_ip:=<robot_host>
```

- Run 3.0.
- **Resolve [DCM ALValue]:** confirm the driver does not error on
  `createAlias`/`setAlias`. If it throws, the real DCM rejected our libqi
  mixed-type list; capture the exact error.
- **Resolve [DCM velocity key]** (the main open item): on the robot, dump the
  actuator subtree and look for a per-joint speed actuator:
  ```sh
  qicli call ALMemory.getDataListName | grep -iE "HeadYaw.*(Speed|Velocity|Actuator)"
  ```
  Candidates to try as `velocity_actuator_key` (with `{}` = joint name):
  `Device/SubDeviceList/{}/Speed/Actuator/Value`,
  `Device/SubDeviceList/{}/ActuatorSpeed/Value`,
  `Device/SubDeviceList/{}/RotationVelocity/Actuator/Value`.
  For each candidate, build a velocity-command URDF and check whether commanding
  a small constant velocity produces steady motion:
  ```sh
  ros2 launch naoqi_driver ros2_control.launch.py robot:=nao \
    plugin:=naoqi_driver/DcmSystem nao_ip:=<host> \
    velocity_actuator_key:='Device/SubDeviceList/{}/Speed/Actuator/Value'
  ```
  **Record** which key (if any) the firmware accepts and moves on.
- **Cross-check** DCM vs ALMotion: same commanded angle should reach the same
  measured angle.

### 3.3 LoLA (`naoqi_driver/LolaSystem`) — NAO 2.8+ (and Pepper 2.9, hidden)

Must run **on the robot**. Validates the socket, MessagePack codec and joint
mapping.

Preconditions on the robot:
- `/home/nao/robocup.conf` exists (may be empty) so the `lola` process creates
  `/tmp/robocup`.
- No other LoLA client (B-Human, etc.) is holding the socket.

```sh
# on the robot
ros2 launch naoqi_driver ros2_control.launch.py \
  robot:=nao plugin:=naoqi_driver/LolaSystem
```

- Run 3.0.
- **Resolve [LoLA wire-format]:** the most likely failure is a decode mismatch.
  If `/joint_states` is empty/garbage, capture one raw frame and compare against
  `lola_protocol.cpp`:
  ```sh
  # on the robot, with our client stopped
  python3 - <<'PY'
  import socket, pprint
  try:
      import msgpack
  except ImportError:
      msgpack = None
  s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect("/tmp/robocup")
  data = s.recv(2048); s.close()
  print("len", len(data))
  if msgpack: pprint.pprint(msgpack.unpackb(data, raw=False))
  PY
  ```
  **Record:** the field names, whether floats are 32- or 64-bit, the `Position`
  array length/order, and whether the server rejects an actuator frame that
  omits LED groups (`Chest`, `LEye`, …). Feed corrections back into
  `encodeActuators`/`decodeSensors` and the LoLA section of `ASSUMPTIONS.md`.
- **Resolve [Pepper LoLA]:** on a Pepper 2.9, check whether `/tmp/robocup`
  exists and what joint order it uses (Pepper's joints differ from NAO; the
  current `jointOrder()` is NAO's).
- **Rate check:** `ros2 topic hz /joint_states` should approach LoLA's ~83 Hz
  if the controller_manager `update_rate` is set high enough.

---

## Assumption → test mapping

Each open item in [`../ASSUMPTIONS.md`](../ASSUMPTIONS.md) is resolved by a
specific step here:

| Assumption | Resolved by | Capture |
|------------|-------------|---------|
| libqi runtime usable from controller_manager | 3.1 launch succeeds | does any backend connect to a real robot at all? |
| DCM accepts libqi mixed-type ALValue | 3.2 | exact error, or success |
| Per-joint DCM velocity key | 3.2 velocity probe | the accepted key, or "none exists" |
| `ClearAll` vs `Merge` smoothness | 3.2 sweep | jitter at command rate? |
| LoLA wire format (fields, float width, framing) | 3.3 frame capture | corrected codec |
| LoLA needs LED groups every frame | 3.3 | does the server stall if omitted? |
| Pepper 2.9 has LoLA; its socket/joint order | 3.3 (Pepper) | socket presence + order |
| ALMotion `setStiffnesses` correctness | 3.1 | holds against gravity? |

## Pass / fail criteria

- **Level 1/2:** all automated tests green; controllers active; `/joint_states`
  live; trajectory goals tracked.
- **Level 3 (per backend):** the robot moves as commanded (correct direction and
  magnitude), state tracks command, no unexpected jerk on start/stop, and the
  assumptions in scope for that backend are recorded as confirmed or corrected.

## Results log (copy per run)

```
date / tester:
robot + NAOqi version:
backend (plugin):           run location (remote / on-robot):
Level 1 tests:              pass / fail (notes)
Level 2 emulation:          pass / fail (notes)
Level 3 head nod:           moved? y/n
Level 3 backend specifics:  (DCM velocity key found? LoLA frame matches? ...)
assumptions confirmed:
assumptions corrected:      (and the code/doc change made)
issues / follow-ups:
```
