# Structure
```
ROS2-Unity-Robotics-Simulator/
├── ros2_docker/
│   └── colcon_ws/
│       └── src/
│           ├── jetsonmouse_description
│           ├── raspimouse_description
│           └── ROS-TCP-Endpoint
├── ROS2UnityRoboticsSimulator/
│   └── Assets/
│       ├── Materials
│       ├── Prefabs
│       ├── Resources
│       ├── Scenes
│       └── Urdf
└── script/
    └── sync_description.sh
```
create by [tree.nathanfriend.id](https://tree.nathanfriend.io/)

# Setup host（Ubuntu on Docker）
Dockerfile is based on [Tiryoh/docker-ros2-desktop-vnc/humble](https://github.com/Tiryoh/docker-ros2-desktop-vnc/tree/master/humble).
## Build image and launch container
1. Build ROS2 docker image if not create image.
```
#cd ros2_docker
#docker build -t [image_name]:[tag_name] .
```
example
```
#docker build -t ros2-decktop-vnc:humble .
```

2. Launch docker container.
```
#docker run --rm -p 10000:10000 -v [host directory]:/[container directory] --shm-size=512m [image_name]:[tag_name]
```
example
```
#docker run --rm -p 10000:10000 -v $HOME/ROS/ROS2-Unity-Robotics-Simulator/ros2_docker/colcon_ws:/home/ubuntu/colcon_ws --shm-size=512m ros2-decktop-vnc:humble
```

If you use docker-compose to run the following commands, you can skip steps 1 and 2.

```
#cd ros2_docker
#docker compose up
```

## Launch ROS-TCP-Endpoint
ROS-TCP-Endpoint is used for communication between ROS and Unity. Enter the container that has been launched and launch ROS-TCP-Endpoint so that ROS and Unity can communicate using the following command.

```
#cd colcon_ws
#colcon build
#source install/setup.bash
#ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## ROS package manager(rosdep)
The packages on which ROS packages depend are described in the `package.xml` of each package. We can install packages that the package depends on by executing `rosdep install`.

example
```
#sudo apt update
#rosdep install -i --from-paths [ros package path]
```

## ROS package build
```
#cd colcon_ws
#colcon build --packages-select [ros package name]
```

## xacro convert to urdf
The xacro file cannot be used by the URDF-Importer and must be converted to a URDF file. The following xacro commands can be used for conversion.

```
#xacro [xacro file] [(option)xacro param] [output urdf file name]
```

example
```
#xacro raspimouse.urdf.xacro use_rgb_camera:=true lidar:=urg > raspimouse_with_camera_urg.urdf
```

Since the URDF for raspimouse_description depends on realsense2_description, the following parts of the exported URDF (raspimouse.urdf, raspimouse_with_camera_lds.urdf, raspimouse_ with_camera_rplidar, raspimouse_with_camera_urg) are changed as follows. This allows the URDF-Importer to resolve dependencies.

```diff
<geometry>
-    <mesh filename="file://ros/humble/share/realsense2_description/meshes/d435.dae"/>
+    <mesh filename="package://realsense2_description/meshes/d435.dae"/>
</geometry>
```

## URDF import for Unity
To import and use URDF on the Unity, use URDF-Importer. We need to copy the ROS2 URDF package and URDF to UnityProject/Assets/Urdf in order to load the URDF using URDF-Importer. Synchronize folders using `sync_description.sh`.If you use another URDF package, add the package name to `PACKAGES_DIR_NAMES` in `sync_description.sh`.

```
#./sync_description.sh
```

The URDF file and package will be copied under UnityProject/Assets/Urdf. Select the URDF file on Unity and execute “Import Robot from selected URDF file”. Then a URDF GameObject is created on the Scene.

# Demos(WIP)
## View camera image published by Unity(Robot)
The RgbCamera script on the Unity converts the Texture to jpeg. The RgbCameraImagePublisher script converts the jpeg to a CompressedImageMsg in the ROS Message and publishes.

The following is a command to subscribe to and view the camera images on the ROS.
```
ros2 run rqt_image_view rqt_image_view compressed "image/compressed:=/camera/rgb/image/compressed"
```
