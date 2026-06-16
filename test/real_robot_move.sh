#!/usr/bin/env bash
#
# Copyright 2025 Aldebaran
#
# Licensed under the Apache License, Version 2.0 (the "License").
#
# Human-in-the-loop test: command a small, safe head motion on a REAL robot
# through the ros2_control stack, so a person can confirm the hardware
# interface actually moves the robot.
#
# This is NOT run in CI (it needs a robot). Run it by hand:
#
#   ./test/real_robot_move.sh <robot_ip> [password] [plugin]
#
#   <robot_ip>   IP or hostname of the robot (required)
#   [password]   NAOqi password (NAOqi >= 2.9). Omit for older robots.
#   [plugin]     Hardware plugin. Default: naoqi_driver/AlMotionSystem
#                Others (when implemented): naoqi_driver/DcmSystem,
#                naoqi_driver/LolaSystem
#
# Prerequisites: the workspace is built and sourced; the robot is reachable;
# there is room around the robot. The script nods the head ~0.3 rad and back.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <robot_ip> [password] [plugin] [robot]" >&2
  echo "  [robot] is 'nao' (default) or 'pepper'" >&2
  exit 2
fi

ROBOT_IP="$1"
PASSWORD="${2:-no_password}"
PLUGIN="${3:-naoqi_driver/AlMotionSystem}"
ROBOT="${4:-nao}"
CONTROLLER="${ROBOT}_joint_trajectory_controller"

echo "============================================================"
echo " REAL ROBOT MOTION TEST"
echo "   robot:   ${ROBOT} @ ${ROBOT_IP}"
echo "   plugin:  ${PLUGIN}"
echo "   motion:  HeadYaw -> +0.3 rad -> 0.0 rad"
echo "============================================================"
echo "Make sure the robot is awake and has room to move its head."
echo "Tip: 'ssh nao@${ROBOT_IP}' then"
echo "     'qicli call ALAutonomousLife.setState disabled' and"
echo "     'qicli call ALMotion.wakeUp' before running this."
read -r -p "Press Enter to start, Ctrl-C to abort... " _

# Bring up controller_manager + controllers against the real robot.
ros2 launch naoqi_driver ros2_control.launch.py \
  robot:="${ROBOT}" \
  emulation_mode:=false \
  plugin:="${PLUGIN}" \
  nao_ip:="${ROBOT_IP}" \
  password:="${PASSWORD}" &
LAUNCH_PID=$!
trap 'kill "${LAUNCH_PID}" 2>/dev/null || true' EXIT

echo "Waiting for ${CONTROLLER} to become active..."
for _ in $(seq 1 30); do
  if ros2 control list_controllers 2>/dev/null | grep -q "${CONTROLLER}.*active"; then
    break
  fi
  sleep 1
done
ros2 control list_controllers || true

echo "Sending head motion goal..."
ros2 action send_goal "/${CONTROLLER}/follow_joint_trajectory" \
  control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [HeadYaw],
    points: [
      { positions: [0.3], time_from_start: { sec: 2 } },
      { positions: [0.0], time_from_start: { sec: 4 } }
    ]
  }
}"

echo "Goal finished. Did the head move? (human verification)"
echo "Shutting down..."
