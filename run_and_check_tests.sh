#!/bin/bash

# This script runs the unit tests and checks the results.
declare file="test_results"
declare regex=" tests, 0 errors, 0 failures, 0 skipped"

# Source ROS2
. /opt/ros/humble/setup.bash

# Build the package
echo "Building the package"
colcon build

# Run the unit tests
echo "Running the unit tests"
colcon test

# Check the results
echo "Checking the results"
colcon test-result > "${file}"
cat "${file}"

declare file_content=$( cat "${file}" )
if [[ " $file_content " =~ $regex ]]; then
    echo "Unit tests passed"
    exit 0
else
    echo "Unit tests failed"
    exit 1
fi
