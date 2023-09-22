
// subscribe human skeleton position (these Position are already transform to the world frame), use multi cameras and average the joint position 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

#include <fstream>

class PointTransformer : public rclcpp::Node
{
public:
    PointTransformer()
    : Node("point_transformer")
    {
        subscriber1_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Nuitrack/cameraT/joints_position", 10, std::bind(&PointTransformer::callback1, this, std::placeholders::_1));
        subscriber2_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Nuitrack/cameraL/joints_position", 10, std::bind(&PointTransformer::callback2, this, std::placeholders::_1));
        subscriber3_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Nuitrack/cameraR/joints_position", 10, std::bind(&PointTransformer::callback3, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Nuitrack/transformed_jointsPosition", 10);
        hand_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Nuitrack/transformed_handsPosition", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&PointTransformer::timer_callback, this));
    }

private:
    void callback1(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, 1);
    }

    void callback2(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, 2);
    }

    void callback3(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, 3);
    }

    void processMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg, int topic_number)
    {
        if (msg->data.size() == 3 && msg->data[0] == 100.0 && msg->data[1] == 100.0) {
            valid_data_received_[topic_number] = false;
            return;
        }

        valid_data_received_[topic_number] = true;

        std::vector<geometry_msgs::msg::Point> points;
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            geometry_msgs::msg::Point point;
            point.x = msg->data[i];
            point.y = msg->data[i + 1];
            point.z = msg->data[i + 2];

            points.push_back(point);
        }
        latest_points_[topic_number] = std::move(points);
    }

    void timer_callback()
    {
        if (!valid_data_received_[1] && !valid_data_received_[2]) {
            std_msgs::msg::Float32MultiArray averaged_msg;
            averaged_msg.data = {100.0, 100.0, 100.0};
            publisher_->publish(averaged_msg);
            return;  // Do not process if all subscribers have no valid data
        }

        std::vector<geometry_msgs::msg::Point> averaged_points;

        for (size_t i = 0; i < latest_points_[1].size(); ++i) {
            geometry_msgs::msg::Point avg_point;
            int valid_count = 0;
            
            double x_sum = 0, y_sum = 0, z_sum = 0;

            // Process each subscriber's data
            for (int sub = 1; sub <= 2; ++sub) {
                if (valid_data_received_[sub] && !std::isnan(latest_points_[sub][i].x)) {
                    x_sum += latest_points_[sub][i].x;
                    valid_count++;
                }
                if (valid_data_received_[sub] && !std::isnan(latest_points_[sub][i].y)) {
                    y_sum += latest_points_[sub][i].y;
                }
                if (valid_data_received_[sub] && !std::isnan(latest_points_[sub][i].z)) {
                    z_sum += latest_points_[sub][i].z;
                }
            }

            // Calculate the averaged point
            avg_point.x = x_sum / valid_count;
            avg_point.y = y_sum / valid_count;
            avg_point.z = z_sum / valid_count;

            averaged_points.push_back(avg_point);
        }

        // Publish averaged points
        std_msgs::msg::Float32MultiArray averaged_msg;
        if (!averaged_points.empty()) {
            
            for (const auto& point : averaged_points) {
                averaged_msg.data.push_back(point.x);
                averaged_msg.data.push_back(point.y);
                averaged_msg.data.push_back(point.z);
            }
            publisher_->publish(averaged_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "No averaged points available to publish.");
        }

        // Create a new message for the desired values
        std_msgs::msg::Float32MultiArray hand_msg;

        // here for Nuitrack is not right, need to change
        if (averaged_msg.data.size() >= 21) {  // Check that there are enough values
            hand_msg.data = {
                averaged_msg.data[15],
                averaged_msg.data[16],
                averaged_msg.data[17],
                averaged_msg.data[18],
                averaged_msg.data[19],
                averaged_msg.data[20]
            };

            // Publish the new message using the hand_publisher_
            hand_publisher_->publish(hand_msg);


            /*
            // ************testing***************************************************************************************

            // Write hand position to a txt file
            std::ofstream outFile;
            outFile.open("Hand_Nuitrack.txt", std::ios::app);  // Open the file in append mode
            outFile << averaged_msg.data[18] << ", " << averaged_msg.data[19] << ", " << averaged_msg.data[20] << std::endl;
            outFile.close();
            RCLCPP_INFO(get_logger(), "Hand position is written to a txt file.");
            // ***********************************************************************************************************
            */

        } else {
            // Optionally handle the case where there aren't enough values
            std::cerr << "Not enough data for hand_msg publication!" << std::endl;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber1_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber2_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber3_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::map<int, std::vector<geometry_msgs::msg::Point>> latest_points_;
    std::map<int, bool> valid_data_received_ = { {1, false}, {2, false}, {3, false} };
    //std::map<int, bool> valid_data_received_ = { {1, false}, {2, false} };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointTransformer>());
    rclcpp::shutdown();
    return 0;
}
