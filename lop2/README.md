
# LOP 2

Run the following commands in the directory of this file.


## Build the Docker image

`docker build -f ./lop2.Dockerfile -t lop2 .`


## Start a Docker container with graphics support

`bash ./start-container.sh`


## Inside the container

### Start the package

In the directory '/ros2_ws/':

Setup ROS:
`source ./install/setup.bash`

Launch the package:
`ros2 launch lop2 main.launch.py`


### If changes to the package are made

Rebuild the workspace:
`colcon build --symlink-install --packages-select lop2`


### Record RealSense topics

Start a RealSense camera node:
Default camera_name is 'camera'.
clip_distance is the max distance for recorded points of the pointcloud,
reduces data usage and raises readability.
`ros2 launch realsense2_camera rs_launch.py depth_module.enable_auto_exposure:=True pointcloud.enable:=True clip_distance:=5.0 camera_name:='CAMERA_NAME'`

Record all topics published by the camera node:
Additionally the transformation topics are recorded.
`ros2 bag record -o BAG_NAME --regex "\/CAMERA_NAME\/.*|\/tf.*"`
Stop the recording with Ctrl+C.

Publish the recorded topics on loop:
read-ahead-queue-size sets the size of the playback buffer, this reduces stuttering.
`ros2 bag play BAG_NAME --loop --read-ahead-queue-size 100`


### Start RealSense camera from command line

`ros2 run realsense2_camera realsense2_camera_node --ros-args -p camera_name:=recCamera -p enable_color:=true -p spatial_filter.enable:=true -p temporal_filter.enable:=true -p pointcloud.enable:=True -p pointcloud.stream_filter:=2 -p align_depth.enable:=True -p enable_accel:=False -p enable_gyro:=False -p enable_infra1:=False -p enable_infra2:=false`


### If 'failed to create symbolic link ... because existing path cannot be removed: Is a directory'

Delete build, install and log directories from workspace and rebuild.
