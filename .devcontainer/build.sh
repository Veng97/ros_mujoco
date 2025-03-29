#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Change to the directory of this script so that relative paths work
cd $(dirname "$0")

# Set the user and group IDs to match the current user
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)

# Build the docker container
docker build \
    --build-arg USER_ID=$USER_ID \
    --build-arg GROUP_ID=$GROUP_ID \
    -t ros-mujoco:latest \
    -f dockerfile .. \
    $@