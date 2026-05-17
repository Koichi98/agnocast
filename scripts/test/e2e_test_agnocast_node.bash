#!/bin/bash

# End-to-end test runner for agnocast::Node (the pure-Agnocast node).
# Runs the 1-to-1 and 2-to-2 launch tests, in which standalone executables
# built on agnocast::Node communicate over Agnocast, driven by Agnocast timers.
#
# When STRESS_TEST_TIMEOUT is set (by e2e_test_stress.bash), it is inherited by
# launch_test: the nodes run with --forever and soak under load for that many
# seconds before the assertions run.

if ! grep -q "^agnocast " /proc/modules; then
    echo "ERROR: agnocast kernel module is not loaded." >&2
    echo "Load it first: sudo insmod agnocast_kmod/agnocast.ko" >&2
    exit 1
fi

# Signal handling: kill all descendant processes on exit
cleanup() {
    trap - SIGINT SIGTERM SIGHUP
    kill -- -$$ 2>/dev/null
    exit 130
}
trap cleanup SIGINT SIGTERM SIGHUP

# Setup
rm -rf build/agnocast_e2e_test install/agnocast_e2e_test
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select agnocast_e2e_test --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

TESTS=(
    "src/agnocast_e2e_test/test/functional/test_agnocast_node_1to1_launch.py"
    "src/agnocast_e2e_test/test/functional/test_agnocast_node_2to2_launch.py"
)

FAILURE_COUNT=0
for test_file in "${TESTS[@]}"; do
    echo "====================== ${test_file} ======================" | sudo tee /dev/kmsg
    launch_test "$test_file"
    if [ $? -ne 0 ]; then
        echo "Test failed: ${test_file}" | sudo tee /dev/kmsg
        FAILURE_COUNT=$((FAILURE_COUNT + 1))
    fi
    # Safety margin before the next test.
    sleep 2
done

if [ "$FAILURE_COUNT" -gt 0 ]; then
    echo "$FAILURE_COUNT / ${#TESTS[@]} agnocast::Node e2e tests failed" | sudo tee /dev/kmsg
    exit 1
else
    echo "All agnocast::Node e2e tests passed!!" | sudo tee /dev/kmsg
fi
