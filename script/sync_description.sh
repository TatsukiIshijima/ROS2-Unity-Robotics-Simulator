#!/bin/bash

ROS2_WS_SRC_DIR="ros2_docker/colcon_ws/src"

# Unity URDF directory for URDF-Importer
DEST_DIR="ROS2UnityRoboticsSimulator/Assets/Urdf"

# ROS2 URDF model packages
PACKAGES_DIR_NAMES=(
    "jetsonmouse_description"
)

if [ ! -d $DEST_DIR ]; then
    echo "Error: $DEST_DIR is not found."
    exit 1
fi

for package in "${PACKAGES_DIR_NAMES[@]}"; do
    source_dir="$ROS2_WS_SRC_DIR/$package"
    if [ -d "$source_dir" ]; then
        echo "sync $source_dir to $DEST_DIR"
        # sync ros2 packages to Unity URDF directory
        rsync -av "$source_dir" "$DEST_DIR/"
        # copy urdf files to Unity URDF directory"
    else
        echo "sync error: $source_dir is not found."
    fi
done