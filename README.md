# Configurations
```
ROS2-Unity-Robotics-Simulator/
├── ros2_docker/
│   └── colcon_ws/
│       └── src/
│           └── ROS-TCP-Endpoint
└── ROS2UnityRoboticsSimulator
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

# Demos(WIP)
## View camera image published by Unity(Robot)
The RgbCamera script on the Unity converts the Texture to jpeg. The RgbCameraImagePublisher script converts the jpeg to a CompressedImageMsg in the ROS Message and publishes.

The following is a command to subscribe to and view the camera images on the ROS.
```
ros2 run rqt_image_view rqt_image_view compressed "image/compressed:=/camera/rgb/image/compressed"
```
