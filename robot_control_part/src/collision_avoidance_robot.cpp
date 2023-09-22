/*
// subscribe the min_distance and then use service call to set the speed of robot
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"
#include "std_srvs/srv/trigger.hpp"


class SpeedSettingSubscriber : public rclcpp::Node
{
public:
  SpeedSettingSubscriber()
  : Node("speed_setting_subscriber")
  {
    // subscriber min_distance
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "/Jointpoints/minimum_distance", 10, std::bind(&SpeedSettingSubscriber::topic_callback, this, std::placeholders::_1));
      //"/pointcloud/minimum_distance", 10, std::bind(&SpeedSettingSubscriber::topic_callback, this, std::placeholders::_1));

    // service cal for speed set
    client_ = this->create_client<ur_msgs::srv::SetSpeedSliderFraction>("/io_and_status_controller/set_speed_slider");
    // Initialize client for the pause service
    pause_client_ = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/pause");

  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    n = min_robot_speed - dist_threshold_low * ((max_robot_speed-min_robot_speed)/(speed_distance-dist_threshold_low));
    if (msg->data >= speed_distance)
    {
      request->speed_slider_fraction = max_robot_speed;
      // request send
      auto result = client_->async_send_request(request);
    }
    else if (msg->data >= dist_threshold_low && msg->data < speed_distance)
    {
      adjusted_speed = ((max_robot_speed-min_robot_speed)/(speed_distance-dist_threshold_low)) * msg->data + n;
      request->speed_slider_fraction = adjusted_speed;

      auto result = client_->async_send_request(request);
    }
    else if (msg->data < dist_threshold_low)
    {
      request->speed_slider_fraction = 0.05f;
      auto result = client_->async_send_request(request);
      
      // this part still with problem, robot can not stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      auto stop_request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto pause_result = pause_client_->async_send_request(stop_request);
      
    }
    
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;

  float max_robot_speed = 0.65;
  float min_robot_speed = 0.2;
  float speed_distance = 1.0;  // max distance for adjusting the robot speed
  float dist_threshold_low = 0.3;
  float dist_threshold_high = 0.5;
  float adjusted_speed = 0.65;
  float n;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SpeedSettingSubscriber>());

  rclcpp::shutdown();
  return 0;
}
*/


// subscribe the min_distance and then use service call to set the speed of robot  ******************can run
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"
#include <std_srvs/srv/trigger.hpp>


class SpeedSettingSubscriber : public rclcpp::Node
{
public:
  SpeedSettingSubscriber()
  : Node("speed_setting_subscriber")
  {
    // subscriber min_distance
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      //"/Jointpoints/minimum_distance", 10, std::bind(&SpeedSettingSubscriber::topic_callback, this, std::placeholders::_1));
      "/pointcloud/minimum_distance", 10, std::bind(&SpeedSettingSubscriber::topic_callback, this, std::placeholders::_1));

    // subscriber handover_flag
    handover_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/handover/approach_flag", 10, std::bind(&SpeedSettingSubscriber::handover_callback, this, std::placeholders::_1));

    // service cal for speed set
    client_ = this->create_client<ur_msgs::srv::SetSpeedSliderFraction>("/io_and_status_controller/set_speed_slider");
    // Initialize client for the pause service
    pause_client_ = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/pause");
    // Initialize client for the start service
    start_client_ = this->create_client<std_srvs::srv::Trigger>("/dashboard_client/play");

    while (rclcpp::ok()) {
      if (handover_flag){
        setSpeed(0.45);
        if (!status){
            startRobot();
        }
      } else if (!near_obstacle) {
        updateSpeed();
      }
      /*
      else if (!handover_flag && monitoring){
        updateSpeed();
      } else if (!handover_flag && !monitoring){
        maxSpeed();
      }
      */

      if (near_obstacle && status && !handover_flag){
        pauseRobot();
      }
      if (!near_obstacle && !status && !handover_flag){
        startRobot();
      }

      rclcpp::spin_some(this->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

  }

private:
  void topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    min_distance = msg->data;
    if (min_distance == 10.0 || min_distance > 1.0){
        min_distance = speed_distance;
    }
    
    if (min_distance <= dist_threshold_low){
        near_obstacle = true;
    }
    if (min_distance >= dist_threshold_high){
        near_obstacle = false;
    }

  }

  void handover_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    handover_flag = msg->data;
  }

  void startRobot(){
    RCLCPP_INFO(this->get_logger(), "Robot Starting");
    auto start_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto start_result = start_client_->async_send_request(start_request);
    status = true;
  }

  void pauseRobot(){
    RCLCPP_INFO(this->get_logger(), "Robot Stopping");
    auto pause_request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto pause_result = pause_client_->async_send_request(pause_request);
    status = false;
  }

  void updateSpeed(){
    RCLCPP_INFO(this->get_logger(), "Robot speed reduce");
    auto request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    n = max_robot_speed - speed_distance * ((max_robot_speed-min_robot_speed)/(speed_distance-dist_threshold_low));
    adjusted_speed = ((max_robot_speed-min_robot_speed)/(speed_distance-dist_threshold_low)) * min_distance + n;
    request->speed_slider_fraction = adjusted_speed;
    auto result = client_->async_send_request(request);
    
  }

  void setSpeed(float wanted_speed){
    auto set_request = std::make_shared<ur_msgs::srv::SetSpeedSliderFraction::Request>();
    set_request->speed_slider_fraction = wanted_speed;
    auto set_result = client_->async_send_request(set_request);
  }
  

  void sleepSafeFor(double duration) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // Adjust the rate as per your requirements

    while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() < duration) {
      rate.sleep();
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr handover_sub_;
  rclcpp::Client<ur_msgs::srv::SetSpeedSliderFraction>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;

  float max_robot_speed = 0.6;
  float min_robot_speed = 0.2;
  float speed_distance = 0.6;  // max distance for adjusting the robot speed
  float dist_threshold_low = 0.23;
  float dist_threshold_high = 0.35;
  float adjusted_speed = 1.0;
  float min_distance;
  float n;
  bool near_obstacle = false;
  bool status = false; // true if robot moving, false if robot stopped
  bool handover_flag = false;
  //bool monitoring = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SpeedSettingSubscriber>());

  rclcpp::shutdown();
  return 0;
}

