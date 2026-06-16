# Assumptions to verify on a real robot

This file records facts and assumptions behind the `ros2_control` hardware
interfaces. Anything marked **[ASSUMPTION]** has *not* been confirmed against a
real robot and must be tested. **[CONFIRMED: src]** items are backed by an
authoritative open-source client or by the existing driver code; they are still
worth re-checking on hardware.

Test harness for confirming these: `test/real_robot_move.sh <ip> [password]`.

## Backend availability matrix

| Robot / NAOqi | State read | Command write | Status |
|---|---|---|---|
| NAO 2.1 | ALMemory keys (~83 Hz) | DCM aliases (pos + stiffness, velocity TBD) | planned |
| Pepper 2.5 | ALMemory keys | DCM aliases (same schema as NAO) | planned |
| NAO 2.8+ | LoLA socket (~83 Hz) | LoLA socket (pos + stiffness) | planned |
| Pepper 2.9 | LoLA socket (hidden API) / ALMemory | LoLA (hidden) or ALMotion | planned |
| any | ALMemory keys | ALMotion `setAngles` (pos) | **implemented** |

- **[ASSUMPTION, expert-asserted]** Pepper 2.9 ships LoLA, just hidden; the API
  is the same as NAO's. Official Aldebaran docs say LoLA is NAO-only — this
  contradiction must be resolved on a Pepper 2.9. Unknown: LoLA socket path on
  Pepper, and Pepper's joint ordering over LoLA (it has different joints + wheels
  than NAO).
- **[ASSUMPTION, expert-asserted]** Pepper 2.5 low-level control works like NAO
  2.1 (DCM). The actuator key *schema* is confirmed identical; what is unverified
  is that the full flow behaves the same on Pepper 2.5.
- **[CONFIRMED]** On NAOqi 2.8+, LoLA refreshes ALMemory at only ~2 Hz (vs ~83 Hz
  before), so fast joint state on 2.8+ must come from the LoLA socket, not from
  reading ALMemory.

## State (sensor) ALMemory keys — used by AlMotionSystem and the joint_state converter

- **[CONFIRMED: driver]** Position: `Device/SubDeviceList/<Joint>/Position/Sensor/Value`
- **[CONFIRMED: driver]** Velocity: `Motion/Velocity/Sensor/<Joint>` (an ALMotion
  key, not a `Device/...` key)
- **[CONFIRMED: driver]** Torque/effort: `Motion/Torque/Sensor/<Joint>`
- **[ASSUMPTION]** When a joint has no velocity/torque key (e.g. hands), ALMemory
  returns void; we map that to 0.0. Confirmed behavior in the fake; assumed on
  hardware.

## DCM command keys and flow (NAO 2.1 / Pepper 2.5) — for the planned DcmSystem

- **[CONFIRMED: mc_naoqi_dcm]** Position command: `Device/SubDeviceList/<Joint>/Position/Actuator/Value`
- **[CONFIRMED: mc_naoqi_dcm]** Stiffness command: `Device/SubDeviceList/<Joint>/Hardness/Actuator/Value`
  (the device key is **"Hardness"**, not "Stiffness")
- **[CONFIRMED: mc_naoqi_dcm]** Flow: `DCM.createAlias([name, [keys...]])` once,
  then per cycle: `t = DCM.getTime(0)` and `DCM.setAlias(cmd)` where `cmd` is a
  size-6 ALValue: `[name, updateType, "time-separate", 0, [t], [[v0],[v1],...]]`.
  `updateType` is `"ClearAll"` (replace) or `"Merge"`.
- Source: https://github.com/jrl-umi3218/mc_naoqi_dcm (NAORobotModule.cpp,
  PepperRobotModule.cpp, mc_naoqi_dcm.cpp)
- **[ASSUMPTION]** `DcmSystem` builds the `createAlias`/`setAlias` arguments as a
  libqi list of mixed-type `AnyValue`s (the qi equivalent of the old SDK's
  `ALValue`). mc_naoqi_dcm used the NAOqi C++ SDK (`AL::ALValue`) directly, not
  libqi. The fake DCM parses our list fine, but that the real DCM accepts a
  qi-messaging mixed list as an ALValue is untested. Verify on hardware; if it is
  rejected, wrap values explicitly or call DCM through a different path.
- **[ASSUMPTION]** `DcmSystem` uses `updateType = "ClearAll"` with a single
  command timed at `getTime(0)` each cycle. Streaming setpoints this way (replace
  each cycle) is assumed smooth enough; a real robot may need `"Merge"` with a
  small future delay. Tune on hardware.

### Velocity via DCM — THE OPEN QUESTION

The maintainer states the DCM exposes per-joint *target speed* keys that ALMotion
uses, and that velocity control is the real goal. Status of the search:

- **[CONFIRMED: mc_naoqi_dcm]** Pepper wheels are velocity actuators:
  `Device/SubDeviceList/Wheel{FL,FR,B}/RotationVelocity/Actuator/Value`. This is
  the confirmed *pattern* for a velocity actuator key.
- **[ASSUMPTION / UNRESOLVED]** A per-joint (arm/leg) velocity actuator key was
  **not** found in open sources. Candidates to probe on a real robot's ALMemory
  tree (`ALMemory.getDataListName()` filtered by joint name):
  - `Device/SubDeviceList/<Joint>/ActuatorSpeed/...`
  - `Device/SubDeviceList/<Joint>/Speed/Actuator/Value`
  - `Device/SubDeviceList/<Joint>/RotationVelocity/Actuator/Value`
  ACTION: on a NAO 2.1 / Pepper 2.5, dump the actuator subtree and identify the
  speed/velocity key the firmware accepts. Until then, DcmSystem will implement
  position+stiffness; velocity command stays unimplemented.

## LoLA protocol (NAO 2.8+) — for the planned LolaSystem

- **[CONFIRMED: ros-sports/nao_lola, B-Human, rUNSWift, HTWK]**
  - Socket: AF_UNIX `/tmp/robocup` (created by the `lola` process when
    `/home/nao/robocup.conf` exists). Serialization: MessagePack map. ~83 Hz / 12 ms.
  - Actuator map (client→lola): `Position`[25 float], `Stiffness`[25 float],
    plus LEDs (`Chest`[3], `LEar`[10], `REar`[10], `LEye`[24], `REye`[24],
    `Skull`[12], `LFoot`[3], `RFoot`[3]).
  - Sensor map (lola→client): `Position`[25], `Stiffness`[25], `Temperature`[25],
    `Current`[25], `Status`[25], `Battery`[4], `Accelerometer`[3], `Gyroscope`[3],
    `Angles`[2], `Sonar`[2], `FSR`[8], `Touch`[14], `RobotConfig`[4 strings].
  - **Joint order (25, NAO)**: HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll,
    LElbowYaw, LElbowRoll, LWristYaw, **LHipYawPitch** (single — L/R coupled),
    LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipRoll, RHipPitch,
    RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll, RElbowYaw,
    RElbowRoll, RWristYaw, LHand, RHand.
- **[ASSUMPTION]** LoLA disables ALMotion's control while a custom client owns the
  socket. The driver's other features (ALMotion-based) may conflict with running
  LolaSystem simultaneously. Decide co-existence policy on hardware.
- **[ASSUMPTION]** Pepper's LoLA joint order, socket path, and sensor/actuator
  field set are unknown (see availability matrix).
- Sources: https://github.com/ros-sports/nao_lola (lola_enums.hpp),
  https://github.com/bhuman/BHumanCodeRelease, https://github.com/NaoHTWK/LolaConnector

### LolaSystem implementation assumptions (verify on a NAO V6)

- **[ASSUMPTION]** The hand-rolled MessagePack codec (`lola_protocol.cpp`)
  matches real LoLA on the wire. It round-trips against the fake server in CI,
  which proves self-consistency, not wire-compatibility. Check field names,
  whether values are float32 vs float64, and map vs array framing on hardware.
- **[ASSUMPTION]** Sending only `Position` + `Stiffness` in the actuator frame is
  accepted. LoLA may require all actuator groups (LED fields) every frame; if so,
  add them to `encodeActuators` (sizes in the table above).
- **[ASSUMPTION]** One `recv()` returns exactly one LoLA frame (no reassembly).
  True for small frames in practice; if LoLA fragments, add length-based framing.
- **[ASSUMPTION]** `RHipYawPitch` shares `LHipYawPitch`'s LoLA slot; commanding
  both writes the same slot (last wins). Correct only because they are coupled.
- **[ASSUMPTION]** LoLA has no joint velocity/torque sensor field, so velocity and
  effort states are left at 0. Could be derived (finite difference) later.

## libqi runtime from inside controller_manager

- **[ASSUMPTION]** `src/hardware/session_factory.cpp` lazily creates a single
  `qi::Application` the first time it connects to a real robot, because a
  pluginlib-loaded hardware interface has no `main()` to own the qi runtime. This
  is untested. If libqi networking fails without an `ApplicationSession`, the
  fallback is to create `qi::Application` in a custom controller_manager
  executable. The emulation path (`createFakeNaoqiSession`) needs none of this.

## ALMotion specifics

- **[CONFIRMED: driver/fake]** `ALMotion.setAngles(names, angles, fractionMaxSpeed)`
  is position control; `fractionMaxSpeed` ∈ (0,1] is a speed *fraction*, not a
  velocity. AlMotionSystem therefore exposes a position command only; velocity is
  state-only.
- **[ASSUMPTION]** `ALMotion.setStiffnesses("Body", 1.0)` on activation is correct
  and safe. The fake does not implement it, so failure there is only warned.
