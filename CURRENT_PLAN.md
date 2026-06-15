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
- `AlMotionSystem` (position) implemented end-to-end:
  - `src/hardware/almotion_system.*`, `src/hardware/session_factory.*`
  - pluginlib export `naoqi_driver_hardware.xml`, CMake/package.xml wiring
  - description `share/ros2_control/nao.urdf.xacro` + reusable
    `naoqi_system.ros2_control.xacro` macro
  - controllers `config/nao_controllers.yaml`, bringup
    `launch/ros2_control.launch.py`
  - emulation gtest `test/almotion_system_test.cpp`
  - human real-robot test `test/real_robot_move.sh`

### Next
1. Resolve the DCM joint-velocity key on a real NAO 2.1 / Pepper 2.5
   (ASSUMPTIONS.md). Until then, no velocity command.
2. `DcmSystem`: `DCM.createAlias` for position + Hardness aliases; per-cycle
   `getTime`/`setAlias`. Add a `FakeDCM` to the fake NAOqi so it is CI-testable.
   Add velocity once the key is known.
3. `LolaSystem`: MessagePack client over `/tmp/robocup` (fields/order in
   ASSUMPTIONS.md). Add a fake LoLA Unix-socket server for CI. Verify Pepper 2.9.
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
