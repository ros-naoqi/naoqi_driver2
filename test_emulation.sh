#!/bin/bash
# Test script for NAOqi driver emulation mode
# This script validates that the fake NAOqi implementation works correctly

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test results
TESTS_PASSED=0
TESTS_FAILED=0

echo "========================================"
echo "NAOqi Driver Emulation Mode Test Suite"
echo "========================================"
echo ""

# Function to print test results
print_result() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓ PASSED${NC}: $2"
        ((TESTS_PASSED+=1))
    else
        echo -e "${RED}✗ FAILED${NC}: $2"
        ((TESTS_FAILED+=1))
    fi
}

abs_delta_exceeds() {
    local a="$1"
    local b="$2"
    local min_delta="$3"
    awk -v a="$a" -v b="$b" -v min="$min_delta" 'BEGIN {d=b-a; if (d<0) d=-d; exit !(d>min)}'
}

# Extract a joint position from two CSV strings:
# - names_csv: "HeadYaw, HeadPitch, ..."
# - pos_csv:   "0.0, 0.1, ..."
get_joint_position_from_csv_lists() {
    local names_csv="$1"
    local pos_csv="$2"
    local joint_name="$3"

    if [ -z "$names_csv" ] || [ -z "$pos_csv" ]; then
        return 1
    fi

    local -a names_arr
    local -a pos_arr
    IFS=',' read -r -a names_arr <<< "$names_csv"
    IFS=',' read -r -a pos_arr <<< "$pos_csv"

    local i
    for i in "${!names_arr[@]}"; do
        local n
        n=$(echo "${names_arr[$i]}" | sed -E 's/^[[:space:]]*//; s/[[:space:]]*$//' | tr -d "'\"[]")
        if [ "$n" = "$joint_name" ]; then
            echo "${pos_arr[$i]}" | sed -E 's/^[[:space:]]*//; s/[[:space:]]*$//' | tr -d "'\"[]"
            return 0
        fi
    done

    return 1
}

# Given a full JointState CSV line (from --csv), extract:
# - names_csv: comma-separated joint names (no header fields)
# - pos_csv:   comma-separated joint positions
# Assumes message is flattened as: header(3 cols) + name(N) + position(N) + velocity(N) + effort(N)
extract_names_and_positions_from_joint_state_flat_csv() {
    local csv_file="$1"

    if [ ! -s "$csv_file" ]; then
        return 1
    fi

    local nf
    nf=$(awk -F',' 'NR==1{print NF; exit}' "$csv_file" 2>/dev/null || true)
    if [ -z "$nf" ]; then
        return 1
    fi

    local header_cols=3
    local remainder=$((nf - header_cols))
    if [ "$remainder" -le 0 ]; then
        return 1
    fi

    # Expect 4 equal-sized arrays after header
    if [ $((remainder % 4)) -ne 0 ]; then
        return 1
    fi
    local n=$((remainder / 4))
    if [ "$n" -le 0 ]; then
        return 1
    fi

    local names_start=4
    local names_end=$((header_cols + n))
    local pos_start=$((header_cols + n + 1))
    local pos_end=$((header_cols + 2*n))

    local names_csv
    local pos_csv
    names_csv=$(cut -d',' -f"${names_start}-${names_end}" "$csv_file" | tr -d '\n')
    pos_csv=$(cut -d',' -f"${pos_start}-${pos_end}" "$csv_file" | tr -d '\n')

    if [ -z "$names_csv" ] || [ -z "$pos_csv" ]; then
        return 1
    fi

    # Print both on separate lines for easy capture.
    echo "$names_csv"
    echo "$pos_csv"
}

# Function to check if process is running
check_process() {
    pgrep -f "$1" > /dev/null
    return $?
}

# Function to cleanup background processes
cleanup() {
    echo ""
    echo "Cleaning up..."
    pkill -f "ros2 launch naoqi_driver" || true
    pkill -f "naoqi_driver_node" || true
    sleep 2
}

on_error() {
    local exit_status="$1"
    print_result 1 "Last command failed, exit status: ${exit_status}"
}

# Trap cleanup on exit; report errors via ERR trap
trap cleanup EXIT
trap 'on_error $?' ERR

# Test 1: Build the package
echo ""
echo "Test 1: Building naoqi_driver package..."
if colcon build --packages-select naoqi_driver; then
    print_result 0 "Package builds successfully"
else
    print_result 1 "Package build failed"
    exit 1
fi

# Test 2: Launch driver in emulation mode (NAO)
echo ""
echo "Test 2: Launching driver in emulation mode (NAO)..."
ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=nao > /tmp/naoqi_test_nao.log 2>&1 &
LAUNCH_PID=$!
sleep 5

# Check if process is still running
if check_process "naoqi_driver_node"; then
    print_result 0 "Driver launches in NAO emulation mode"

    # Test 3: Check NAOqi version is correct for NAO
    echo ""
    echo "Test 3: Verifying NAOqi version for NAO..."
    if grep -q "2\.8\.6\.23" /tmp/naoqi_test_nao.log; then
        print_result 0 "NAO reports correct NAOqi version (2.8.6.23)"
    else
        print_result 1 "NAO version incorrect"
    fi

    # Test 4: Check all services are registered
    echo ""
    echo "Test 4: Checking fake NAOqi services registration..."
    EXPECTED_SERVICES="ALMotion ALMemory ALTextToSpeech ALVideoDevice ALAudioDevice ALSonar ALBodyTemperature ALRobotModel ALSystem ALDialog ALSpeechRecognition LogManager"
    ALL_REGISTERED=true

    for service in $EXPECTED_SERVICES; do
        if ! grep -q "Registered Service \"$service\"" /tmp/naoqi_test_nao.log; then
            echo "  Missing service: $service"
            ALL_REGISTERED=false
        fi
    done

    if $ALL_REGISTERED; then
        print_result 0 "All fake NAOqi services registered"
    else
        print_result 1 "Some services missing"
    fi

    # Test 5: Check ROS topics are published
    echo ""
    echo "Test 5: Checking ROS topics availability..."
    sleep 2  # Give time for topics to be advertised

    EXPECTED_TOPICS="/joint_states /joint_angles /cmd_vel /odom /diagnostics"
    TOPICS_OK=true

    for topic in $EXPECTED_TOPICS; do
        if ! timeout 2 ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
            echo "  Missing topic: $topic"
            TOPICS_OK=false
        fi
    done

    if $TOPICS_OK; then
        print_result 0 "Expected ROS topics are available"
    else
        print_result 1 "Some ROS topics missing"
    fi

    # Test 6: Test joint_states topic is publishing
    echo ""
    echo "Test 6: Publishing /joint_angles and verifying /joint_states updates..."

    # Capture a baseline joint state
    timeout 3 ros2 topic echo /joint_states --once --csv > /tmp/joint_states_before.csv
    if [ ! -s /tmp/joint_states_before.csv ]; then
        print_result 1 "Failed to capture baseline /joint_states (empty or missing)"
        exit 1
    fi

    # Publish a joint command
    echo "Publishing /joint_angles command..."
    timeout 3 ros2 topic pub --once /joint_angles naoqi_bridge_msgs/msg/JointAnglesWithSpeed "{header: {stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5, 0.1], speed: 0.1, relative: 0}"

    # Read joint states again and check HeadYaw/HeadPitch changed
    echo "Capturing /joint_states after publishing..."
    timeout 3 ros2 topic echo /joint_states --once --csv > /tmp/joint_states_after.csv
    if [ ! -s /tmp/joint_states_after.csv ]; then
        print_result 1 "Failed to capture baseline /joint_states (empty or missing)"
        exit 1
    fi

    NAMES_BEFORE=""
    POS_BEFORE=""
    NAMES_AFTER=""
    POS_AFTER=""

    if ! readarray -t BEFORE_LINES < <(extract_names_and_positions_from_joint_state_flat_csv /tmp/joint_states_before.csv 2>/dev/null); then
        print_result 1 "CSV parse failed for /joint_states baseline"
        echo "  Baseline CSV (first 300 chars):"
        head -c 300 /tmp/joint_states_before.csv 2>/dev/null || true
        echo "  Publish output (for debugging):"
        tail -50 /tmp/joint_angles_pub.txt || true
    elif ! readarray -t AFTER_LINES < <(extract_names_and_positions_from_joint_state_flat_csv /tmp/joint_states_after.csv 2>/dev/null); then
        print_result 1 "CSV parse failed for /joint_states after publish"
        echo "  After CSV (first 300 chars):"
        head -c 300 /tmp/joint_states_after.csv 2>/dev/null || true
        echo "  Publish output (for debugging):"
        tail -50 /tmp/joint_angles_pub.txt || true
    else
        NAMES_BEFORE="${BEFORE_LINES[0]:-}"
        POS_BEFORE="${BEFORE_LINES[1]:-}"
        NAMES_AFTER="${AFTER_LINES[0]:-}"
        POS_AFTER="${AFTER_LINES[1]:-}"

        BEFORE_YAW=$(get_joint_position_from_csv_lists "$NAMES_BEFORE" "$POS_BEFORE" HeadYaw || true)
        BEFORE_PITCH=$(get_joint_position_from_csv_lists "$NAMES_BEFORE" "$POS_BEFORE" HeadPitch || true)
        AFTER_YAW=$(get_joint_position_from_csv_lists "$NAMES_AFTER" "$POS_AFTER" HeadYaw || true)
        AFTER_PITCH=$(get_joint_position_from_csv_lists "$NAMES_AFTER" "$POS_AFTER" HeadPitch || true)

        # If CSV is usable, these must be present.
        if [ -z "$AFTER_YAW" ] || [ -z "$AFTER_PITCH" ]; then
            print_result 1 "CSV did not contain HeadYaw/HeadPitch"
            echo "  CSV extraction (for debugging):"
            echo "    names_before: ${NAMES_BEFORE:0:200}"
            echo "    pos_before:   ${POS_BEFORE:0:200}"
            echo "    names_after:  ${NAMES_AFTER:0:200}"
            echo "    pos_after:    ${POS_AFTER:0:200}"
            echo "  Publish output (for debugging):"
            tail -50 /tmp/joint_angles_pub.txt || true
        else
            MIN_DELTA=0.02
            OK=true

            if [ -z "$BEFORE_YAW" ]; then BEFORE_YAW="$AFTER_YAW"; fi
            if [ -z "$BEFORE_PITCH" ]; then BEFORE_PITCH="$AFTER_PITCH"; fi

            if ! abs_delta_exceeds "$BEFORE_YAW" "$AFTER_YAW" "$MIN_DELTA"; then
                OK=false
            fi
            if ! abs_delta_exceeds "$BEFORE_PITCH" "$AFTER_PITCH" "$MIN_DELTA"; then
                OK=false
            fi

            if $OK; then
                print_result 0 "/joint_states updated after publishing /joint_angles"
            else
                print_result 1 "/joint_states did not update after publishing /joint_angles"
                echo "  Extracted positions (before -> after):"
                echo "    HeadYaw:   ${BEFORE_YAW:-<missing>} -> ${AFTER_YAW:-<missing>}"
                echo "    HeadPitch: ${BEFORE_PITCH:-<missing>} -> ${AFTER_PITCH:-<missing>}"
                echo "  CSV extraction (for debugging):"
                echo "    names_before: ${NAMES_BEFORE:0:200}"
                echo "    pos_before:   ${POS_BEFORE:0:200}"
                echo "    names_after:  ${NAMES_AFTER:0:200}"
                echo "    pos_after:    ${POS_AFTER:0:200}"
                echo "  Publish output (for debugging):"
                tail -50 /tmp/joint_angles_pub.txt || true
            fi
        fi
    fi

else
    print_result 1 "Driver failed to launch in NAO emulation mode"
    cat /tmp/naoqi_test_nao.log
fi

# Cleanup NAO test
cleanup
sleep 2

# Test 7: Launch driver in emulation mode (Pepper)
echo ""
echo "Test 7: Testing Pepper emulation mode..."
ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=pepper > /tmp/naoqi_test_pepper.log 2>&1 &
sleep 5

if check_process "naoqi_driver_node"; then
    print_result 0 "Driver launches in Pepper emulation mode"

    # Test 8: Check NAOqi version is correct for Pepper
    echo ""
    echo "Test 8: Verifying NAOqi version for Pepper..."
    if grep -q "2\.9\.5\.3" /tmp/naoqi_test_pepper.log; then
        print_result 0 "Pepper reports correct NAOqi version (2.9.5.3)"
    else
        print_result 1 "Pepper version incorrect"
        grep "NAOqi" /tmp/naoqi_test_pepper.log || true
    fi
else
    print_result 1 "Driver failed to launch in Pepper emulation mode"
fi

cleanup
sleep 2

# Test 9: Launch driver in emulation mode (Romeo)
echo ""
echo "Test 9: Testing Romeo emulation mode..."
ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=romeo > /tmp/naoqi_test_romeo.log 2>&1 &
sleep 5

if check_process "naoqi_driver_node"; then
    print_result 0 "Driver launches in Romeo emulation mode"

    # Test 10: Check NAOqi version is correct for Romeo
    echo ""
    echo "Test 10: Verifying NAOqi version for Romeo..."
    if grep -q "2\.1\.4\.13" /tmp/naoqi_test_romeo.log; then
        print_result 0 "Romeo reports correct NAOqi version (2.1.4.13)"
    else
        print_result 1 "Romeo version incorrect"
        grep "NAOqi" /tmp/naoqi_test_romeo.log || true
    fi
else
    print_result 1 "Driver failed to launch in Romeo emulation mode"
fi

# Print summary
echo ""
echo "========================================"
echo "Test Summary"
echo "========================================"
echo -e "Tests passed: ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Tests failed: ${RED}${TESTS_FAILED}${NC}"
echo ""

# Exit with appropriate code
if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed.${NC}"
    echo ""
    echo "Logs are available in:"
    echo "  - /tmp/naoqi_test_nao.log"
    echo "  - /tmp/naoqi_test_pepper.log"
    echo "  - /tmp/naoqi_test_romeo.log"
    exit 1
fi
