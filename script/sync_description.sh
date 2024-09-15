#!/bin/bash

ROS2_WS_SRC_DIR="ros2_docker/colcon_ws/src"

# Unity URDF directory for URDF-Importer
DEST_DIR="ROS2UnityRoboticsSimulator/Assets/Urdf"

# ROS2 URDF model packages
# if you want to add more packages, add package name to PACKAGES_DIR_NAMES
PACKAGES_DIR_NAMES=(
    "jetsonmouse_description"
    "raspimouse_description"
    "realsense2_description"
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
        # extract urdf files from subdirectories
        find "$DEST_DIR/$package" -type f -name "*.urdf" -exec rsync -av {} "$DEST_DIR/" \;
    else
        echo "sync error: $source_dir is not found."
    fi
done