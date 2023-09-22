// publish robot joints position
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <std_msgs/msg/float32_multi_array.hpp> // 1. Include the header

class RobotLinkPositionListener : public rclcpp::Node
{
public:
    RobotLinkPositionListener()
    : Node("robot_link_position_listener")
    {
        // 2. Create a publisher
        joint_positions_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Robot/joint_positions", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RobotLinkPositionListener::timer_callback, this));
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    }

private:
    /*
    void timer_callback()
    {
        auto current_state = move_group_interface_->getCurrentState();
        const std::vector<std::string>& link_names = move_group_interface_->getLinkNames();
        
        // Create Float32MultiArray message to store joint positions
        std_msgs::msg::Float32MultiArray joint_positions_msg;
        
        for(const auto& link_name : link_names)
        {
            const Eigen::Isometry3d& link_state = current_state->getGlobalLinkTransform(link_name);
            Eigen::Vector3d link_position = link_state.translation();
            
            // Print the link name and its XYZ position
            RCLCPP_INFO(this->get_logger(), "Link: %s, Position: X: %f, Y: %f, Z: %f", 
                        link_name.c_str(), link_position.x(), link_position.y(), link_position.z());

            // Populate the Float32MultiArray message
            joint_positions_msg.data.push_back(link_position.x());
            joint_positions_msg.data.push_back(link_position.y());
            joint_positions_msg.data.push_back(link_position.z());
        }

        // 3. Publish the Float32MultiArray message
        joint_positions_publisher_->publish(joint_positions_msg);
    }
    */

    void timer_callback()
    {
        auto current_state = move_group_interface_->getCurrentState();
        const std::vector<std::string> interested_links = {"upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"};
        std::map<std::string, Eigen::Vector3d> link_positions;
        std_msgs::msg::Float32MultiArray joint_positions_msg;

        for(const auto& link_name : interested_links)
        {
            const Eigen::Isometry3d& link_state = current_state->getGlobalLinkTransform(link_name);
            Eigen::Vector3d link_position = link_state.translation();
            link_positions[link_name] = link_position;
            
            // Populate the Float32MultiArray message
            joint_positions_msg.data.push_back(link_position.x());
            joint_positions_msg.data.push_back(link_position.y());
            joint_positions_msg.data.push_back(link_position.z());
        }

        // Compute the direction from wrist_2_link to wrist_3_link
        Eigen::Vector3d direction = (link_positions["wrist_3_link"] - link_positions["wrist_2_link"]).normalized();

        // Create a new point that is 18 cm from wrist_3_link in the direction of wrist_3_link
        Eigen::Vector3d new_point = link_positions["wrist_3_link"] + (direction * 0.18);

        // Add new point to the message
        joint_positions_msg.data.push_back(new_point.x());
        joint_positions_msg.data.push_back(new_point.y());
        joint_positions_msg.data.push_back(new_point.z());

        // Publish the Float32MultiArray message
        joint_positions_publisher_->publish(joint_positions_msg);
    }


    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_positions_publisher_; // Publisher declaration
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotLinkPositionListener>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
