#include <memory>
#include <iostream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ur_msgs/srv/set_io.hpp>
#include <ur_msgs/srv/set_speed_slider_fraction.hpp>


#include <moveit_msgs/msg/move_it_error_codes.hpp>




class UR10EMoveit : public rclcpp::Node {
public:
  UR10EMoveit(const std::shared_ptr<rclcpp::Node>& node, const std::string& move_group_name) 
  : Node("ur10e_moveit"), move_group_interface_(node, move_group_name) {
    
  // Convert Euler angles to quaternion
  tf2::Quaternion q;
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

  // Initialize the service client for setting IO
  set_io_client_ = create_client<ur_msgs::srv::SetIO>("/io_and_status_controller/set_io");

  // Initialize the service client for speed set
  speed_client_ = create_client<ur_msgs::srv::SetSpeedSliderFraction>("/io_and_status_controller/set_speed_slider");
  
  stop_client_ = create_client<std_srvs::srv::Trigger>("/dashboard_client/pause");

  // Begin state machine
  //timer_ = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&UR10EMoveit::stateMachine, this));
  while (rclcpp::ok()) {
      stateMachine();
      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

private:

  void stateMachine() {
    std::cout << "Doing nominal Task" << std::endl;
    /*
    // set the speed of robot 
    auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    request->speed_slider_fraction = 0.4f;
    auto result = speed_client_->async_send_request(request);
    */
    open_gripper();
    switcher_ = !switcher_;
    moveBetweenFixedPoints((switcher_) ? point1_pose_ : point2_pose_);
    sleepSafeFor(2.0);
    close_gripper();
    sleepSafeFor(3.0);

    /*
    auto stop_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto stop_result = stop_client_->async_send_request(stop_request);
    
    stop_result.wait();  // This will wait until there's a response or timeout (depends on your configuration).
        if (stop_result.valid())
        {
            auto response = stop_result.get();
            if (!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to stop robot: %s", response->message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get response from pause service");
        }
    */
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
    const double eef_step = 0.1;
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







/*
  bool open_gripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_)
  {
    ur_msgs::srv::SetIO::Request set_req1;
    set_req1.fun = 1;
    set_req1.pin = 16;
    set_req1.state = 1.0;
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "%s", ur_msgs::srv::to_yaml(set_req1).c_str());
    auto fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(set_req1));
    auto fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    ur_msgs::srv::SetIO::Request reset_req2;
    reset_req2.fun = 1;
    reset_req2.pin = 17;
    reset_req2.state = 0.0;

    fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(reset_req2));
    fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    return true;
  }

  bool close_gripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_)
  {
    ur_msgs::srv::SetIO::Request set_req2;
    set_req2.fun = 1;
    set_req2.pin = 17;
    set_req2.state = 1.0;

    RCLCPP_INFO(rclcpp::get_logger("set_io"), "%s", ur_msgs::srv::to_yaml(set_req2).c_str());
    auto fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(set_req2));
    auto fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    ur_msgs::srv::SetIO::Request reset_req1;
    reset_req1.fun = 1;
    reset_req1.pin = 16;
    reset_req1.state = 0.0;

    fut =
        set_io_client_->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(reset_req1));
    fut_res = fut.wait_for(std::chrono::seconds(2));
    if (fut_res == std::future_status::timeout)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
      return false;
    }

    if (!fut.get()->success)
    {
      RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
      return false;
    }

    return true;
  }
*/


  void sleepSafeFor(double duration) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // Adjust the rate as per your requirements
    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
      rate.sleep();
    }
  }


  geometry_msgs::msg::Pose point1_pose_;
  geometry_msgs::msg::Pose point2_pose_;
  bool switcher_ = false;
  bool ToolInGripper = false;
  geometry_msgs::msg::Pose pose;
  
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;
  rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr speed_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  
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