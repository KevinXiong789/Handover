How to start Roboter with Openpose:

Every time when vscode new open
Terminal #1 unset GTK_PATH

Every Terminal, run code about ur_driver and ur_description, should run this source code:
Terminal #1 source ~/san/robot_control/install/setup.bash 
Terminal #1 source install/setup.bash

Terminal #1 ros2 launch lop2 main.launch.py 

--> Control Panel Connect to IP Adress in the Teach Panel

Terminal #2 ros2 launch robot_control_part handover_statemachine.launch.py 
Terminal #3 ros2 run pointcloud_processing handover_T_inhand_detector 



!!!!!!!!!!! SSM Model      !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    How to start SSM Model (min distance between skeleton and robot):
        T #1 ros2 launch lop2 main.launch.py
        --> Control Panel Connect to IP Adress in the Teach Panel
        --> In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh
        --> IP and Port: 172.21.19.98 and 29999
        --> Send Message play
        T #2 ros2 launch robot_control_part pickplace_test.launch.py 

        make sure in "min_distance.cpp" in "robot_control_part" pkg build, there are two programs in this cpp (second one)
        T #3 ros2 run robot_control_part min_distance  
        T #4 ros2 run robot_control_part collision_avoidance_robot

    How to start SSM Model (min distance between right hand point cloud and robot):
        T #1 ros2 launch lop2 main.launch.py
        --> Control Panel Connect to IP Adress in the Teach Panel
        --> In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh
        --> IP and Port: 172.21.19.98 and 29999
        --> Send Message play
        T #2 ros2 launch robot_control_part pickplace_test.launch.py 
        T #3 ros2 run pointcloud_processing handover_T_inhand_detector 

        make sure in "min_distance.cpp" in "robot_control_part" pkg build, there are two programs in this cpp (first one)
        T #4 ros2 run robot_control_part min_distance  
        also in "collision_avoidance_robot.cpp" in line 16 do some change
        T #5 ros2 run robot_control_part collision_avoidance_robot


How to start Roboter with Nuitrack:

Every time when vscode new open
Terminal #1 unset GTK_PATH

Every Terminal, run code about ur_driver and ur_description, should run this source code:
Terminal #1 source ~/san/robot_control/install/setup.bash 

Terminal #1 ros2 launch nuitrack_pointcloud nuitrack.launch.py 

--> Control Panel Connect to IP Adress in the Teach Panel

Terminal #2 ros2 launch robot_control_part handover_statemachine.launch.py 
Terminal #3 ros2 run pointcloud_processing_nuitrack handover_T_inhand_detector 

    How to start SSM Model (min distance between skeleton and robot):
        T #1 ros2 launch nuitrack_pointcloud nuitrack.launch.py 
        --> Control Panel Connect to IP Adress in the Teach Panel
        --> In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh
        --> IP and Port: 172.21.19.98 and 29999
        --> Send Message play
        T #2 ros2 launch robot_control_part pickplace_test.launch.py 

        make sure in "min_distance.cpp" in "robot_control_part" pkg build, there are two programs in this cpp (second one)
        and line 159 change to line 160
        T #3 ros2 run robot_control_part min_distance  
        T #4 ros2 run robot_control_part collision_avoidance_robot





!!!!!!!!!!!!!!!!!!!!!New Version!!!!!!!!!!!!!!!!!!!!
How to start Roboter with Openpose:

Every time when vscode new open
Terminal #1 unset GTK_PATH

Every Terminal, run code about ur_driver and ur_description, should run this source code:
Terminal #1 source ~/san/robot_control/install/setup.bash 
Terminal #1 source install/setup.bash

Terminal #1 ros2 launch lop2 main.launch.py 

--> Control Panel Connect to IP Adress in the Teach Panel
--> In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh
--> IP and Port: 172.21.19.98 and 29999
--> Send Message play

Terminal #2 ros2 launch robot_control_part ssm.launch.py 
Terminal #3 ros2 run pointcloud_processing handover_T_inhand_detector 
Terminal #4 ros2 launch robot_control_part handover_statemachine.launch.py 




How to start Roboter with Nuitrack:

Every time when vscode new open
Terminal #1 unset GTK_PATH

Every Terminal, run code about ur_driver and ur_description, should run this source code:
Terminal #1 source ~/san/robot_control/install/setup.bash 
Terminal #1 source install/setup.bash

Terminal #1 ros2 launch nuitrack_pointcloud nuitrack.launch.py 

--> Control Panel Connect to IP Adress in the Teach Panel
--> In Panel switch to remote control, in computer ~/Downloads: sh SocketTest.sh
--> IP and Port: 172.21.19.98 and 29999
--> Send Message play

Terminal #2 ros2 launch robot_control_part ssm.launch.py 
#(go to pointcloud_processing handover_grasping_detector.cpp line 27 use nuitrack)

Terminal #3 ros2 run pointcloud_processing_nuitrack handover_T_inhand_detector 
Terminal #4 ros2 launch robot_control_part handover_statemachine.launch.py 


