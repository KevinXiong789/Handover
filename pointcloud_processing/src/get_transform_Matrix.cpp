/*
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_listener");

  tf2_ros::Buffer tfBuffer(node->get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);

  rclcpp::WallRate rate(1.0);
  while (rclcpp::ok()){
    geometry_msgs::msg::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("world", "recCameraR_depth_frame", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(node->get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    // 提取xyz偏移量
    double x = transformStamped.transform.translation.x;
    double y = transformStamped.transform.translation.y;
    double z = transformStamped.transform.translation.z;

    // 转换四元数到欧拉角
    tf2::Quaternion q(
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z,
      transformStamped.transform.rotation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // 打印结果
    RCLCPP_INFO(node->get_logger(), "Translation: [%.7f, %.7f, %.7f]", x, y, z);
    RCLCPP_INFO(node->get_logger(), "Rotation: [Roll: %.8f, Pitch: %.8f, Yaw: %.8f]", roll, pitch, yaw);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}
*/


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp" // Replace with the appropriate message types

#include <chrono>
#include <fstream>
#include <string>

class TopicHzCalculator : public rclcpp::Node
{
public:
    TopicHzCalculator()
    : Node("topic_hz_calculator")
    {
        // Subscribe to three topics
        subscription1_ = this->create_subscription<std_msgs::msg::String>(
            "/recCamera/depth/color/points", 10, std::bind(&TopicHzCalculator::topic_callback1, this, std::placeholders::_1));
            
        subscription2_ = this->create_subscription<std_msgs::msg::String>(
            "/recCameraR/depth/color/points", 10, std::bind(&TopicHzCalculator::topic_callback2, this, std::placeholders::_1));
            
        subscription3_ = this->create_subscription<std_msgs::msg::String>(
            "/recCameraL/depth/color/points", 10, std::bind(&TopicHzCalculator::topic_callback3, this, std::placeholders::_1));
            
        // Set up a timer to calculate Hz every second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TopicHzCalculator::calculate_hz, this));
    }

private:
    void topic_callback1(const std_msgs::msg::String::SharedPtr msg)
    {
        (void)msg;
        msg_count1_++;
    }

    void topic_callback2(const std_msgs::msg::String::SharedPtr msg)
    {
        (void)msg;
        msg_count2_++;
    }
    
    void topic_callback3(const std_msgs::msg::String::SharedPtr msg)
    {
        (void)msg;
        msg_count3_++;
    }

    void calculate_hz()
    {
        // Calculate Hz for three topics
        double hz1 = msg_count1_;
        double hz2 = msg_count2_;
        double hz3 = msg_count3_;
        
        RCLCPP_INFO(this->get_logger(), "Hz Topic 1: %f, Hz Topic 2: %f, Hz Topic 3: %f", hz1, hz2, hz3);

        // Reset message counts
        msg_count1_ = 0;
        msg_count2_ = 0;
        msg_count3_ = 0;

        // Write hz values to their respective files
        write_to_file("hz_output_cameraT.txt", hz1);
        write_to_file("hz_output_cameraR.txt", hz2);
        write_to_file("hz_output_cameraL.txt", hz3);
    }
    
    void write_to_file(const std::string& filename, double hz)
    {
        std::ofstream file(filename, std::ios_base::app); // Appending to file
        if (file.is_open())
        {
            file << hz << "\n";
            file.close();
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unable to open file %s for writing.", filename.c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription2_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription3_;
    rclcpp::TimerBase::SharedPtr timer_;
    int msg_count1_ = 0;
    int msg_count2_ = 0;
    int msg_count3_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopicHzCalculator>());
    rclcpp::shutdown();
    return 0;
}


