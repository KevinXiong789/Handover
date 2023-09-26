# Robot Human Collaboration
Here are pkgs in Seher project.

This repo is the source code of the software for a human robot collaboration demonstrator. HW: A UR10e robot and 3 Realsense D435 cameras SW environment: Linux Ubuntu 22.04 and ROS Humble The demonstrator has following functions:
* Construct and represent the occupancy map (point cloud) of the workcells in Rviz based on the camera sensors.
* Calculate the minimum distance between each joint of the robot and the occupancy map
* Based on the minimum distance, the robot slows down or stops if the human is too close to the robot.
* With the help of human skeleton tracker (lightweight Openpose and Nuitrack, we have tested with these two trackers), the position of the human hand can be tracked
* An autonomous tool handover between the robot and the human and vice-versa can be performed, when triggered by the human.
* The behaviour of the robot is based on a state machine.
* The ROS2 system is connected to the cloud database InfluxDB. Data can be sent and received.

## Operating environment and necessary installation
Before this repository can be used properly, the following need to be installed:
* Ubuntu 22.04 + ROS [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* Realsense D435 camera + Intelrealsense SDK + [Intelrealsense Wrapper](https://github.com/IntelRealSense/realsense-ros) (**"realsense-ros"** should be located in **"your workspace/src"** as other packages in this repo)
* [lightweight Openpose](https://github.com/Daniil-Osokin/lightweight-human-pose-estimation.pytorch) code are already realized in **lop2**
* [Nuitrack install](https://github.com/3DiVi/nuitrack-sdk) + for [python beta](https://github.com/3DiVi/nuitrack-sdk/blob/master/PythonNuitrack-beta/README.MD)
* [UR Robot Driver install](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) (better build from source and put the pkg **"robot_control"** in **"your workspace/src"**)
* [InflxDB Download](https://portal.influxdata.com/downloads/) and [InfluxDB-client](https://www.influxdata.com/blog/getting-started-python-influxdb/) for python *(and you also need to have your own InfluxDB account)*

## How to run this repo
The launch files to start the camera and connect the robot using Openpose and Nuitrack are **lop2/lop2/launch/main.launch.py** and **nuitrack_pointcloud/launch/nuitrack.launch.py** respectively. After that, you need to switch the robot to remote mode and run the programme in URCap, then run the specified launch file to set the robot speed, and then start the launch file for the state machine for the robot to start moving.
### Use lightweight Openpose as Tracker
In workspace, Terminal 1:
`ros2 launch lop2 main.launch.py`

Connect with real Robot:
* Load the programme in contro Panel
* Control Panel Connect to IP Adress in the Teach Panel
* In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh
* IP and Port: 172.21.19.98 and 29999
* Send Message play

Terminal 2:
`ros2 launch robot_control_part ssm.launch.py`

Terminal 3:
`ros2 run pointcloud_processing handover_T_inhand_detector`

Terminal 4:
`ros2 launch robot_control_part handover_statemachine.launch.py `

### Use Nuitrack as Tracker
In workspace, Terminal 1:
`ros2 launch nuitrack_pointcloud nuitrack.launch.py`

Connect with real Robot:
* Load the programme in contro Panel
* Control Panel Connect to IP Adress in the Teach Panel
* In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh (this file is located on the computer downstairs next to the robot)
* IP and Port: 172.21.19.98 and 29999
* Send Message play

Terminal 2: (go to pointcloud_processing handover_grasping_detector.cpp line 27 use nuitrack)
`ros2 launch robot_control_part ssm.launch.py`

Terminal 3:
`ros2 run pointcloud_processing_nuitrack handover_T_inhand_detector`

Terminal 4:
`ros2 launch robot_control_part handover_statemachine.launch.py`
