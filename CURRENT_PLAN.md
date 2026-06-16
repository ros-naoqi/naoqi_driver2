# Plan: ros2_control for NAO and Pepper

## Goal

Expose NAO/Pepper joints through `ros2_control` so standard controllers
(joint_state_broadcaster, joint_trajectory_controller, forward controllers) can
drive the robot. Cover every NAOqi generation by selecting the right transport,
and keep everything testable without a robot via the in-process fake NAOqi from
the `fake_naoqi` study this branch builds on.

## Design

`ros2_control`'s unit is a `SystemInterface` plugin loaded by
`controller_manager`. The three transports to the robot are genuinely different
concerns, so each is its own plugin, selected in the URDF `<hardware><plugin>`
(no internal backend switching). They share connection parameters; each ignores
the ones it does not need.

| Plugin | Transport | Targets | Command | Runs |
|---|---|---|---|---|
| `naoqi_driver/AlMotionSystem` | libqi → ALMotion | NAOqi 2.1–2.9 (universal) | position | anywhere |
| `naoqi_driver/DcmSystem` | libqi → DCM + ALMemory | NAO 2.1, Pepper 2.5 | position (+velocity TBD) | anywhere |
| `naoqi_driver/LolaSystem` | `/tmp/robocup` socket | NAO 2.8+, Pepper 2.9 (hidden) | position | on the robot |

Shared pieces:
- `src/hardware/session_factory.*` — builds a libqi session (real or fake) from
  ros2_control params. Reused by ALMotion and DCM systems.
- State is read from ALMemory with the same keys as the `joint_state` converter.

Velocity: none of the transports offer a native joint velocity *command*
(ALMotion's speed is a fraction; LoLA/DCM are position+stiffness). Position is
the baseline. Velocity is pursued through DCM speed keys — see ASSUMPTIONS.md,
"Velocity via DCM", which is the main open research item.

## Status

### Done
- Branch based on the `fake_naoqi` study (fake ALMemory/ALMotion/services,
  emulation mode, ALMemory-key state path).
- Shared `src/hardware/`:
  - `session_factory.*` — libqi session (real/fake) from ros2_control params
  - `joint_state_keys.*` — ALMemory state keys + batched read, used by both
    libqi backends
- `AlMotionSystem` (position) — `almotion_system.*`, `ALMotion.setAngles`.
- `DcmSystem` (position + stiffness; velocity ready behind a key param) —
  `dcm_system.*`, DCM `createAlias`/`getTime`/`setAlias`. Each joint declares
  one command interface (position XOR velocity); velocity needs the
  `velocity_actuator_key` param. `FakeDCM` added to the fake NAOqi.
- `LolaSystem` (position + stiffness) — `lola_system.*`, AF_UNIX `/tmp/robocup`,
  hand-rolled MessagePack codec (`lola_protocol.*`, no msgpack dependency), a
  background IO thread decoupling the 83 Hz LoLA cycle from controller_manager,
  and URDF<->LoLA 25-joint mapping (RHipYawPitch aliases LHipYawPitch). Tested
  against a fake LoLA Unix-socket server (`test/fake_lola_server.hpp`).
- Wiring: pluginlib export (`naoqi_driver_hardware.xml`, all three classes), reusable
  `naoqi_system.ros2_control.xacro` macro + `nao.urdf.xacro` (plugin selectable),
  `config/nao_controllers.yaml`, `launch/ros2_control.launch.py`.
- Tests: emulation gtests `test/almotion_system_test.cpp`,
  `test/dcm_system_test.cpp`; human real-robot `test/real_robot_move.sh`
  (works with any plugin via its 3rd arg).

### Next (real-robot validation)
1. Verify the hand-rolled MessagePack matches real LoLA on a NAO V6 (field
   names, float widths, whether LoLA requires LED groups in every actuator
   frame). Confirm `/tmp/robocup` and that LoLA exists/socket path on Pepper 2.9.
2. Resolve the DCM per-joint velocity key on a real NAO 2.1 / Pepper 2.5, then
   set `velocity_actuator_key` and validate velocity command (ASSUMPTIONS.md).
3. Verify on a real robot that a mixed-type libqi list is accepted by the real
   DCM as an ALValue (the emulator parses it; hardware is untested).
4. Pepper description (`pepper.urdf.xacro`) with its joint set (+ wheels via a
   separate velocity interface / cmd_vel).
5. Decide co-existence between a running naoqi_driver node and a ros2_control
   backend (both talk to the robot).

## How to test

```bash
# Emulation (no robot): unit test
colcon test --packages-select naoqi_driver --ctest-args -R naoqi_driver_hardware_test

# Emulation: full stack
ros2 launch naoqi_driver ros2_control.launch.py emulation_mode:=true
ros2 control list_controllers
ros2 topic echo /joint_states --once

# Real robot (human verification)
./src/naoqi_driver/test/real_robot_move.sh <robot_ip> [password]
```
