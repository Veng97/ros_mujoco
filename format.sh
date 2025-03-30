#!/bin/bash

set -e

# Get the directory where the script is located
SEARCH_DIR=$(dirname "$(realpath "$0")")

# Define the folders to search for files
FOLDERS=("ros_mujoco" "ros_mujoco_interfaces")

# Gather all files in the specified folders with the extensions [.cpp, .h, .hpp]
FILES=$(find "${FOLDERS[@]/#/$SEARCH_DIR/}" -type f \( -iname "*.cpp" -o -iname "*.h" -o -iname "*.hpp" \))

# Run clang-format on the gathered files
clang-format --verbose -i --style=file $FILES
