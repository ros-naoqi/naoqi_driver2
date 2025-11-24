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
        ((TESTS_PASSED++))
    else
        echo -e "${RED}✗ FAILED${NC}: $2"
        ((TESTS_FAILED++))
    fi
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

# Trap cleanup on exit
trap cleanup EXIT

# Source ROS environment
echo "Setting up ROS environment..."
source /opt/ros/iron/setup.bash
source install/setup.bash

# Test 1: Build the package
echo ""
echo "Test 1: Building naoqi_driver package..."
if colcon build --packages-select naoqi_driver 2>&1 | grep -q "Finished <<<"; then
    print_result 0 "Package builds successfully"
else
    print_result 1 "Package build failed"
    exit 1
fi

# Test 2: Launch driver in emulation mode (NAO)
echo ""
echo "Test 2: Launching driver in emulation mode (NAO)..."
timeout 10 ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=nao > /tmp/naoqi_test_nao.log 2>&1 &
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
    echo "Test 6: Testing /joint_states publishes data..."
    if timeout 3 ros2 topic echo /joint_states --once > /tmp/joint_states.txt 2>&1; then
        if grep -q "name:" /tmp/joint_states.txt && grep -q "position:" /tmp/joint_states.txt; then
            print_result 0 "/joint_states publishes valid data"
        else
            print_result 1 "/joint_states data format invalid"
        fi
    else
        print_result 1 "/joint_states not publishing (this is expected until subscriber fix)"
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
timeout 10 ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=pepper > /tmp/naoqi_test_pepper.log 2>&1 &
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
timeout 10 ros2 launch naoqi_driver naoqi_driver.launch.py emulation_mode:=true robot_type:=romeo > /tmp/naoqi_test_romeo.log 2>&1 &
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
