// robot state machine, nominal takeaway give, state machine not in subscriber loop
// update hand position                                                               ******************************************************************can run
#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ur_msgs/srv/set_io.hpp>

enum RobotState {
  NOMINAL,
  BEFORETAKEAWAY,
  TAKEAWAY,
  PICKTOOL,
  PLACETOOL,
  GIVE
};

class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {
    
	robot_state = NOMINAL;
	updateStatus();

    // Convert Euler angles to quaternion
  tf2::Quaternion q;
	// yaw manuall offset 30 degrees
  double roll = 0, pitch = M_PI, yaw = 0.52359878;  // All in radians
  q.setRPY(roll, pitch, yaw);

  point1_pose_.orientation.w = q.getW();
  point1_pose_.orientation.x = q.getX();
  point1_pose_.orientation.y = q.getY();
  point1_pose_.orientation.z = q.getZ();

  point1_pose_.position.x = 0.2;
  point1_pose_.position.y = 0.6;
  point1_pose_.position.z = 0.4;

  // point2 for nominal task
  point2_pose_ = point1_pose_;
  point2_pose_.position.x = -0.2;

	toolPoint_pose_ = point1_pose_;
	toolPoint_pose_.position.x = 0.08;
	toolPoint_pose_.position.y = 0.4;
	toolPoint_pose_.position.z = 0.15;

  toolPoint_approach_pose_ = toolPoint_pose_;
  toolPoint_approach_pose_.position.z = 0.25;

  move_group_interface_.setGoalPositionTolerance(0.001);
  move_group_interface_.setGoalOrientationTolerance(0.01);
  move_group_interface_.setGoalJointTolerance(0.001);
  move_group_interface_.allowReplanning(true);

    
  // Declare a subscriber for the hand position topic
	hand_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/handover/tool_grasping_position", 1, std::bind(&UR10EMoveit::handPositionCallback, this, std::placeholders::_1));

  // Declare a subscriber for the handover_trigger bool topic
	handover_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/handover/handover_trigger", 1, std::bind(&UR10EMoveit::handoverTriggerCallback, this, std::placeholders::_1));

  // Declare a subscriber for the ToolInHandFlag bool topic
	ToolInHandFlag_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/handover/ToolInHandFlag", 1, std::bind(&UR10EMoveit::toolInHandCallback, this, std::placeholders::_1));

  // Declare a publisher to publish flag when robot doing handover
  handover_pub_ = create_publisher<std_msgs::msg::Bool>("/handover/approach_flag", 1);

  // Initialize the service client for setting IO
  set_io_client_ = create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

  grasp_sub_ = create_subscription<std_msgs::msg::Bool>(
		"/handover/grasp_flag", 1, std::bind(&UR10EMoveit::graspCallback, this, std::placeholders::_1));

    

	// Begin state machine
	//timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UR10EMoveit::stateMachine, this));
	while (rclcpp::ok()) {
      stateMachine();
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

private:
/*
  void handoverTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!handoverTriggerFlag_ && msg->data) {
      triggered_handover_ = true;
    }
    handoverTriggerFlag_ = msg->data;
  }
*/

  void handoverTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	  triggered_handover_ = msg->data;
  }

  
/*
  bool previous_msg_ = false;
  int false_count_ = 0;   

  void handoverTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data && !previous_msg_) {
      // Message transitioned from false to true
      triggered_handover_ = true;
      false_count_ = 0; // Reset the false count since we got a true value
    } else if (!msg->data) {
      // Message is false
      false_count_++;
      if (false_count_ > 30) {
      // Message has been false more than 10 times
      triggered_handover_ = false;
      false_count_ = 0; // Reset the false count since it has been triggered
      }
    } else {
      // Message remains true, so we reset the false count
      false_count_ = 0;
    }

    previous_msg_ = msg->data; // Store the current value for next time
	}
*/



  void toolInHandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	  ToolInHandFlag_ = msg->data;
  }

  void handPositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::cout << "New hand position received" << std::endl;
    handPosition_ = msg;
  }

  void graspCallback(const std_msgs::msg::Bool::SharedPtr msg) {
	  graspFlag_ = msg->data;
  }

  
  void updateStatus() {
	if (triggered_handover_ && robot_state == NOMINAL) {
		  if (ToolInHandFlag_) {
			  robot_state = BEFORETAKEAWAY;
		  } else {
			  robot_state = PICKTOOL;
			}
	} else if (robot_state == PICKTOOL) {
		  robot_state = GIVE;
	} else if (robot_state == GIVE) {
      if (graspFlag_){
          open_gripper();//*********************************
          sleepSafeFor(1.0);
          robot_state = NOMINAL;
      } else {
          robot_state = PLACETOOL;
      }
      
  } else if (robot_state == BEFORETAKEAWAY) {
      robot_state = TAKEAWAY;
  } else if (robot_state == TAKEAWAY) {
      if (graspFlag_){
          close_gripper();//*********************************
          sleepSafeFor(1.0);
          robot_state = PLACETOOL;
      } else {
          robot_state = NOMINAL;
      }

  } else {
		  robot_state = NOMINAL;
	}
  }


  void stateMachine() {
    std_msgs::msg::Bool flag;
    updateStatus();
    switch (robot_state) {
      case NOMINAL:
        std::cout << "Doing nominal Task" << std::endl;
        flag.data = false;
		    handover_pub_->publish(flag);

        switcher_ = !switcher_;
        moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
        sleepSafeFor(5.0);
        break;

      case BEFORETAKEAWAY:
        std::cout << "Move to approach point wait for grasping" << std::endl;
        flag.data = true;
        handover_pub_->publish(flag);
        
        pose = getApproachPosition(handPosition_);
        printf("hand position xyz: %f,%f,%f\n",pose.position.x,pose.position.y,pose.position.z);
        if (isPoseWithinRange(pose)) {
          
          open_gripper();//*********************************

          moveToGraspPosition(pose);
          sleepSafeFor(1.0);

          //close_gripper();//*********************************
          //sleepSafeFor(3.0);

        } else {
          std::cout << "Pose is out of range!" << std::endl;
          break;
        }
        //triggered_handover_ = false;
        break;

      case TAKEAWAY:
        std::cout << "Grasping tool from human hand" << std::endl;
        flag.data = true;
        handover_pub_->publish(flag);
        
        pose = getGraspPosition(handPosition_);
        printf("hand position xyz: %f,%f,%f\n",pose.position.x,pose.position.y,pose.position.z);
        if (isPoseWithinRange(pose)) {
          
          open_gripper();//*********************************

          // first move to approach point
          pose.position.y -= 0.1;
          moveToGraspPosition(pose);
          //sleepSafeFor(1.0);

          // then move to grasping point
          pose.position.y += 0.1;
          moveToGraspPosition(pose);
          sleepSafeFor(1.0);

          //close_gripper();//*********************************
          //sleepSafeFor(3.0);

        } else {
          std::cout << "Pose is out of range!" << std::endl;
          break;
        }
        //triggered_handover_ = false;
        break;

      case PICKTOOL:
        std::cout << "Move to tool cell to pick tool" << std::endl;
        flag.data = false;
        handover_pub_->publish(flag);
        
        open_gripper();//********************************

        PickPlaceTool(toolPoint_approach_pose_);
        //sleepSafeFor(1.0);

        PickPlaceTool(toolPoint_pose_);
        //sleepSafeFor(1.0);
        close_gripper();//*******************************
        std::cout << "Robot moved to tool cell to pick successfully" << std::endl;
        
        sleepSafeFor(1.0);
        PickPlaceTool(toolPoint_approach_pose_);

        //ToolInGripper = true;
        //sleepSafeFor(3.0);
        //triggered_handover_ = false;
        break;

      case PLACETOOL:
        std::cout << "Move to tool cell to place tool" << std::endl;
        flag.data = false;
        handover_pub_->publish(flag);

        PickPlaceTool(toolPoint_approach_pose_);
        //sleepSafeFor(1.0);

        PickPlaceTool(toolPoint_pose_);
        //sleepSafeFor(1.0);
        open_gripper();//********************************
        std::cout << "Robot moved to tool cell to place successfully" << std::endl;
        
        sleepSafeFor(1.0);
        PickPlaceTool(toolPoint_approach_pose_);

        //ToolInGripper = true;
        //sleepSafeFor(1.0);
        triggered_handover_ = false;
        break;

      case GIVE:
        std::cout << "Give tool to human hand" << std::endl;
        flag.data = true;
        handover_pub_->publish(flag);
        
        pose = getGivePosition(handPosition_);
        printf("hand position xyz: %f,%f,%f\n",pose.position.x,pose.position.y,pose.position.z);
        if (isPoseWithinRange(pose)) {
          // first move to approach point
          pose.position.y -= 0.1;
          moveToGivePosition(pose);
          //sleepSafeFor(1.0);
          
          // then move to give point
          pose.position.y += 0.1;
          moveToGivePosition(pose);
          sleepSafeFor(4.0);

          //open_gripper();//***********************************
          //sleepSafeFor(3.0);


        } else {
          std::cout << "Pose is out of range!" << std::endl;
          break;
        }
        //ToolInGripper = false;
        triggered_handover_ = false;
        break;
    }
  }


  void moveBetweenFixedPoints(const geometry_msgs::msg::Pose& point_pose) {
    int trial = 0;
    while(trial < 20) {
      moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
      if (moveGroupExecutePlan(plan)) {
        std::cout << "Robot moved to fixed point successfully" << std::endl;
        return;
      }
      std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
    }
    std::cerr << "Max execution attempts reached, error" << std::endl;
    sleepSafeFor(1.0);
  }

  moveit::planning_interface::MoveGroupInterface::Plan getCartesianPathPlanToPose(const geometry_msgs::msg::Pose& point_pose) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    //waypoints.push_back(move_group_interface_.getCurrentPose().pose);
    waypoints.push_back(point_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.2;
    const double jump_threshold = 0.0;
    double fraction = 0.0;

    int trial = 0;
    while(fraction < 0.5 && trial++ < 5) {
      fraction = move_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    }
    
    if(trial == 5 && fraction < 0.5) {
      std::cerr << "Could not compute cartesian path for given waypoints, aborting!!" << std::endl;
      exit(-1);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    return plan;
  }

  bool moveGroupExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    return move_group_interface_.execute(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }


  bool isPoseWithinRange(const geometry_msgs::msg::Pose& pose) {
    return pose.position.x >= -0.5 && pose.position.x <= 0.5 &&
      pose.position.y >= 0.4 && pose.position.y <= 0.88 &&
      pose.position.z >= 0.25 && pose.position.z <= 1.0;
  }



  void PickPlaceTool(const geometry_msgs::msg::Pose& point_pose) {
    int trial = 0;
    while(trial < 20) {
      moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
      if (moveGroupExecutePlan(plan)) {
      //std::cout << "Robot moved to tool cell to place successfully" << std::endl;
      return;
      }
      std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
    }
    std::cerr << "Max execution attempts reached, error" << std::endl;
    sleepSafeFor(1.0);
  }

/*
  void PickTool(const geometry_msgs::msg::Pose& point_pose) {
    int trial = 0;
    while(trial < 20) {
      moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
      if (moveGroupExecutePlan(plan)) {
        std::cout << "Robot moved to tool cell to pick successfully" << std::endl;
        return;
      }
      std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
    }
    std::cerr << "Max execution attempts reached, error" << std::endl;
    sleepSafeFor(1.0);
  }
*/

  void open_gripper()
  {
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = 1;
    request->pin = 17;
    request->state = 0.0;
    set_io_client_->async_send_request(request);

    request->pin = 16;
    request->state = 1.0;
    set_io_client_->async_send_request(request);
  }

  void close_gripper()
  {
    auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
    request->fun = 1;
    request->pin = 16;
    request->state = 0.0;
    set_io_client_->async_send_request(request);

    request->pin = 17;
    request->state = 1.0;
    set_io_client_->async_send_request(request);
  }


  void sleepSafeFor(double duration) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
      rate.sleep();
    }
  }

  
  geometry_msgs::msg::Pose getApproachPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = msg->data[0];
    pose.position.y = 0.45;
    pose.position.z = msg->data[2];
    tf2::Quaternion q;
      double roll = -M_PI/2, pitch = M_PI - 2.0943951 - msg->data[3], yaw = 0;  // All in radians
      q.setRPY(roll, pitch, yaw);

      pose.orientation.w = q.getW();
      pose.orientation.x = q.getX();
      pose.orientation.y = q.getY();
      pose.orientation.z = q.getZ();
    
    return pose;
  }


  geometry_msgs::msg::Pose getGraspPosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = msg->data[0];
    pose.position.y = msg->data[1] - 0.2;
    //pose.position.y = 0.65;
    pose.position.z = msg->data[2];
    tf2::Quaternion q;
      double roll = -M_PI/2, pitch = M_PI - 2.0943951 - msg->data[3], yaw = 0;  // All in radians
      q.setRPY(roll, pitch, yaw);

      pose.orientation.w = q.getW();
      pose.orientation.x = q.getX();
      pose.orientation.y = q.getY();
      pose.orientation.z = q.getZ();
    
    return pose;
  }

  geometry_msgs::msg::Pose getGivePosition(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = msg->data[0] + 0.03;
    pose.position.y = msg->data[1] - 0.25;
    //pose.position.y = 0.65;
    pose.position.z = msg->data[2] + 0.15;  // offset in z to better give
    tf2::Quaternion q;
      double roll = -M_PI/2, pitch = M_PI - 2.0943951, yaw = 0;  // All in radians
      q.setRPY(roll, pitch, yaw);

      pose.orientation.w = q.getW();
      pose.orientation.x = q.getX();
      pose.orientation.y = q.getY();
      pose.orientation.z = q.getZ();
    
    return pose;
  }



  void moveToGraspPosition(const geometry_msgs::msg::Pose& point_pose) {
    int trial = 0;
    while(trial < 20) {
      moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
      if (moveGroupExecutePlan(plan)) {
      std::cout << "Robot moved to grasping point successfully" << std::endl;
      return;
      }
      std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
    }
    std::cerr << "Max execution attempts reached, error" << std::endl;
    sleepSafeFor(1.0);
  }



  void moveToGivePosition(const geometry_msgs::msg::Pose& point_pose) {
    int trial = 0;
    while(trial < 20) {
      moveit::planning_interface::MoveGroupInterface::Plan plan = getCartesianPathPlanToPose(point_pose);
      if (moveGroupExecutePlan(plan)) {
        std::cout << "Robot moved to tool place successfully" << std::endl;
        return;
      }
      std::cerr << "Execution to point trial " << trial++ << " failed. Reattempting" << std::endl;
    }
    std::cerr << "Max execution attempts reached, error" << std::endl;
    sleepSafeFor(1.0);
  }

/*
  // do not use Cartesian path planning for give position (plan can be very dangerous and long, every time different)
  void moveToGivePosition(geometry_msgs::msg::Pose pose) {
    // Code to move the robot to the given position based on the received message
    if (robot_state != GIVE) {
      std::cout << "Invalid robot state for moveToGivePosition" << std::endl;
      return;
    }

    std::cout << "Give tool to human hand" << std::endl;

    // Set the target pose in the MoveGroup interface
    move_group_interface_.setPoseTarget(pose);
    // Create a plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    if (move_group_interface_.plan(plan1)) {
      move_group_interface_.execute(plan1);
      std::cout << "Robot moved to give point successfully" << std::endl;
    } else {
      std::cerr << "Failed to plan trajectory to target pose" << std::endl;
      return;
    }
  }
*/


  geometry_msgs::msg::Pose point1_pose_;
  geometry_msgs::msg::Pose point2_pose_;
  geometry_msgs::msg::Pose toolPoint_pose_;
  geometry_msgs::msg::Pose toolPoint_approach_pose_;
  RobotState robot_state;
  bool switcher_ = false;
  bool ToolInGripper = false;
  geometry_msgs::msg::Pose pose;
  
  bool ToolInHandFlag_;
  bool handoverTriggerFlag_ = false;
  bool triggered_handover_ = false;
  bool graspFlag_ = false;
  std_msgs::msg::Float32MultiArray::SharedPtr handPosition_;
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handover_trigger_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ToolInHandFlag_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr handover_pub_;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasp_sub_;

};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto move_group_name = "ur_manipulator";

  auto moveit_node = std::make_shared<UR10EMoveit>(node, move_group_name);
  rclcpp::spin(moveit_node);
  rclcpp::shutdown();
  return 0;
}
