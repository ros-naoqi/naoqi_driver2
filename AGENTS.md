# Guidelines for AI Agents Working on naoqi_driver

## Core Principles

### Interaction Style

- **Concise**: Provide brief, to-the-point responses focusing on essential information. Avoid qualifiers like in "modular architecture", "robust implementation", "comprehensive testing".
Describe things and we'll assess quality ourselves. You're not selling us something.
- **Scientific**: Use precise, technical language. Avoid vague terms and generalizations. Focus on facts, data, and specific implementation details.
- **Developers are Roboticits**: Assume the reader has a background in robotics and software development. Use terminology and concepts familiar to roboticists without over-explaining basic ideas.

### Code Style Conformance

- **Follow existing patterns**: Before making changes, examine similar code in the repository to understand the established patterns
- **C++ style**: Use existing indentation (2 spaces), naming conventions, and code organization
- **ROS 2 conventions**: Follow ROS 2 best practices for node structure, topic naming, parameter handling, and service definitions
- **Namespace usage**: Respect the `naoqi` namespace and existing module organization

### Code Organization Understanding

- **Read README.md first**: The README contains essential information about how the driver works, its dependencies, and usage patterns
- **Entry points**: `external_registration.cpp` is the main entry point; `naoqi_driver.cpp` contains the core driver logic
- **Modular structure**: Code is organized into converters, publishers, subscribers, services, recorders, and events
- **NAOqi integration**: The driver uses libQi to connect to NAOqi services via `qi::Session`

### Planning and Documentation

- **Use CURRENT_PLAN.md**: For complex multi-step tasks, maintain a plan document that:
  - Breaks down work into clear subtasks
  - Tracks progress and completion status
  - Provides context for resuming work after disconnection
  - Gets updated at significant milestones
- **Update README.md**: Add information that helps humans understand:
  - High-level architecture and code organization
  - Topic semantics and ROS conventions used
  - Usage examples and common patterns
- **Update AGENTS.md**: Add information that helps AI agents:
  - Technical implementation details discovered during work
  - Service interfaces and their expected behaviors
  - Gotchas and common pitfalls

### ROS 2 Best Practices

- **Topic semantics**: Each ROS topic has specific meaning and expected message format:
  - Research on ros.org or ROS wiki when unclear
  - Document findings for future reference
  - Respect standard topic names (e.g., `/cmd_vel`, `/joint_states`)
- **Parameters**: Use ROS 2 parameter system for configuration
- **Launch files**: Support both Python (.launch.py) and XML (.launch) formats where they exist
- **Node lifecycle**: Respect ROS 2 node initialization, running, and shutdown patterns
- **Workspaces**: The ROS 2 workspace (e.g., `/ws`) is arranged as follows:
  - Sources are cloned under the folder `src` (e.g., `/ws/src/naoqi_driver`)
  - `colcon build` is run from the root of the workspace (`/ws`)
  - Build artifacts go into `build`, `install`, and `log` folders at the workspace root.
  - To run artifacts, source the script `install/setup.bash` from the workspace root (e.g. `/ws/install/setup.bash`)

### Investigation Strategy

1. **Search before implementing**: Use grep_search and semantic_search to find existing patterns
2. **Read related code**: Understand context before making changes
3. **Check dependencies**: Be aware of naoqi_libqi, naoqi_libqicore, and naoqi_bridge_msgs
4. **Test incrementally**: Build and verify changes in logical chunks
5. **Use absolute paths**: Always use absolute paths in terminal commands (e.g., `/ws/src/...` not `src/...`) to avoid directory confusion

### Commands (Time-Savers)

All commands are meant to be run from the root of the ROS 2 workspace (e.g. `/ws`).
All commands assume you have sourced the ROS 2 environment (e.g. `source /opt/ros/${ROS_DISTRO}/setup.bash`).

```bash
# Fast iterative build (build only naoqi_driver)
colcon build --packages-select naoqi_driver

# Build with minimal output (see only errors/summary)
colcon build --packages-select naoqi_driver 2>&1 | tail -50

# Full workspace build
colcon build

# Clean build (when things go wrong)
rm -rf build install log && colcon build

# Test emulation mode (automated test suite)
source install/setup.bash && ./src/naoqi_driver/test_emulation.sh

# Quick manual test - NAO emulation (5 second timeout)
source install/setup.bash && timeout 5 ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=nao

# Quick manual test - Pepper emulation
source install/setup.bash && timeout 5 ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=pepper

# Check available ROS topics
ros2 topic list

# Read joint states once (verify it's publishing)
ros2 topic echo /joint_states --once

# Publish to joint_angles (test motion commands)
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5, 0.1], speed: 0.1, relative: 0}"

# Search for code patterns
grep -r "pattern" src/naoqi_driver --include="*.cpp" --include="*.hpp"

# Find files by name
find src/naoqi_driver -name "*keyword*"

# Check recent build errors
tail -100 log/latest_build/naoqi_driver/stderr.log
```

### Working Efficiently

- **Batch reads**: Read multiple file sections in parallel when gathering context
- **Targeted searches**: Use specific search terms rather than broad queries
- **Multi-file edits**: Use multi_replace_string_in_file for related changes across files
- **Clear commits**: Group related changes logically
- **Absolute paths in commands**: Always start paths from root (/) in terminal commands

### Favorite Commands for This Repo

```bash
# Build just naoqi_driver (fast iteration)
cd /ws && colcon build --packages-select naoqi_driver

# Launch in emulation mode
cd /ws && source /opt/ros/iron/setup.bash && source install/setup.bash && ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=nao

# List all topics
ros2 topic list

# Echo a topic once to see its format
ros2 topic echo /joint_states --once

# Monitor joint states continuously
ros2 topic echo /joint_states

# Publish to joint_angles
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5, 0.1], speed: 0.1, relative: 0}"

# Check topic publish rate
ros2 topic hz /joint_states

# Find files by pattern
find /ws/src/naoqi_driver -name "*.cpp" | grep -i publisher

# Search for string in code
grep -r "joint_states" /ws/src/naoqi_driver/src --include="*.cpp" --include="*.hpp"

# Check build errors
tail -100 /ws/log/latest_build/naoqi_driver/stdout_stderr.log
```

## NAOqi-Specific Knowledge

### Session Management
- The driver connects to NAOqi via `qi::Session` using libQi
- Services are accessed via `session->service("ServiceName")`
- The session can connect to remote NAOqi or run locally (for emulation)

### Key NAOqi Services (to be expanded during implementation)
- **ALMotion**: Motor control, joint angles, trajectories
- **ALMemory**: Event system, memory keys
- **ALTextToSpeech**: Speech synthesis
- **ALAudioDevice**: Audio streaming
- (Add more as discovered)

### Joint States and Motion
- `/joint_states` publishes current robot joint positions (ROS standard)
- Joint positions should reflect commanded positions from motion commands
- Trajectories should be played back realistically over time

## Common Pitfalls

- Don't assume all NAOqi services are available on all robot types (NAO vs Pepper)
- Be careful with ROS time vs system time
- Session must listen on network for audio callbacks (or disable audio in emulation)
- Password triggers TLS and different port (9503 instead of 9559)

## Task-Specific Notes
(Add notes here as you work on specific features or discover important details)
