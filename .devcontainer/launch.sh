#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Change to the directory of this script so that relative paths work
cd $(dirname "$0")

# Allow the container to connect to the X server
xhost +local:

# Launch the container
docker compose -f docker-compose.yml up -d
docker exec -it ros-mujoco-dev bash