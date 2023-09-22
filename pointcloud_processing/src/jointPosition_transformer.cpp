/*
// transform the joint position from camera frame to world frame, just use one camera to get the joint position
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class TransformPointsNode : public rclcpp::Node
{
public:
    TransformPointsNode() : Node("transform_points_node"), buffer_(this->get_clock()), tf2_listener_(buffer_)
    {
        hand_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("transformed_handPosition", 10);
        all_points_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("transformed_jointsPosition", 10);
        subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/lop_recCamera/limb_joints_position",
            10,
            std::bind(&TransformPointsNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 2)
        {
            //RCLCPP_INFO(this->get_logger(), "No person in the area.");
            std_msgs::msg::Float32MultiArray flag_msg;
            flag_msg.data = {100.0, 100.0};
            all_points_publisher_->publish(flag_msg);
            return;
        }

        try
        {
            const auto transform_stamped = buffer_.lookupTransform("world", "recCamera_color_optical_frame", tf2::TimePointZero);
            tf2::Transform transform;
            tf2::fromMsg(transform_stamped.transform, transform);

            std_msgs::msg::Float32MultiArray transformed_msg;
            transformed_msg.data.resize(msg->data.size());

            for (size_t i = 0; i < msg->data.size(); i += 3)
            {
                if (std::isnan(msg->data[i]) && std::isnan(msg->data[i + 1]) && std::isnan(msg->data[i + 2]))
                {
                    transformed_msg.data[i] = std::nan("");
                    transformed_msg.data[i + 1] = std::nan("");
                    transformed_msg.data[i + 2] = std::nan("");
                    continue;
                }

                tf2::Vector3 point(msg->data[i], msg->data[i + 1], msg->data[i + 2]);
                point = transform * point;

                transformed_msg.data[i] = point.x();
                transformed_msg.data[i + 1] = point.y();
                transformed_msg.data[i + 2] = point.z();
            }
            // here are all limb points, keypoint 0 to 8
            all_points_publisher_->publish(transformed_msg);

            // Create a message for the hand position (9th to 14th values, 3 pairs)
            // 9 - 11 are elbow, 12 - 14 are wrist
            std_msgs::msg::Float32MultiArray hand_msg;
            for (size_t i = 9; i < 15; i++) {
                hand_msg.data.push_back(transformed_msg.data[i]);
            }

            hand_publisher_->publish(hand_msg);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform points: %s", ex.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr all_points_publisher_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPointsNode>());
    rclcpp::shutdown();
    return 0;
}
*/



/*
// transform the joint position from camera frame to world frame, try to use two cameras    **********************testing
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <vector>
#include <numeric>

class TransformPointsNode : public rclcpp::Node
{
public:
    TransformPointsNode() 
        : Node("transform_points_node"), buffer_(this->get_clock()), tf2_listener_(buffer_)
    {
        hand_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("transformed_handPosition", 10);
        all_points_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("transformed_jointsPosition", 10);

        transformed_points_.assign(24, 0.0);
        point_count_.assign(24, 0);

        subscriber1_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/lop_recCameraR/limb_joints_position", 10, 
            std::bind(&TransformPointsNode::callback1, this, std::placeholders::_1));

        subscriber2_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/lop_recCamera/limb_joints_position", 10, 
            std::bind(&TransformPointsNode::callback2, this, std::placeholders::_1));
    }

private:
    void processMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg, const std::string &frame_id)
    {
        try 
        {
            const auto transform_stamped = buffer_.lookupTransform("world", frame_id, tf2::TimePointZero);
            tf2::Transform transform;
            tf2::fromMsg(transform_stamped.transform, transform);

            for (size_t i = 0; i < msg->data.size(); i += 3) 
            {
                if (!std::isnan(msg->data[i]) && !std::isnan(msg->data[i + 1]) && !std::isnan(msg->data[i + 2])) 
                {
                    tf2::Vector3 point(msg->data[i], msg->data[i + 1], msg->data[i + 2]);
                    point = transform * point;

                    transformed_points_[i] += point.x();
                    transformed_points_[i + 1] += point.y();
                    transformed_points_[i + 2] += point.z();

                    point_count_[i]++;
                    point_count_[i + 1]++;
                    point_count_[i + 2]++;
                }
            }
            
            publishAveragedData();
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to transform points: %s", ex.what());
        }
    }

    void callback1(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, "recCameraR_color_optical_frame");
    }

    void callback2(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, "recCamera_color_optical_frame");
    }

    void publishAveragedData()
    {
        std_msgs::msg::Float32MultiArray averaged_msg;
        for (size_t i = 0; i < transformed_points_.size(); i++) 
        {
            if (point_count_[i] > 0)
                averaged_msg.data.push_back(transformed_points_[i] / point_count_[i]);
            else
                averaged_msg.data.push_back(std::nan(""));

            // reset for next set of data
            transformed_points_[i] = 0;
            point_count_[i] = 0;
        }

        all_points_publisher_->publish(averaged_msg);

        std_msgs::msg::Float32MultiArray hand_msg;
        for (size_t i = 9; i < 15; i++) {
            hand_msg.data.push_back(averaged_msg.data[i]);
        }

        hand_publisher_->publish(hand_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber1_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber2_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr all_points_publisher_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_listener_;

    std::vector<float> transformed_points_;
    std::vector<int> point_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPointsNode>());
    rclcpp::shutdown();
    return 0;
}
*/



// transform the joint position from camera frame to world frame, use multi cameras and average the joint position 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

#include <fstream>


class PointTransformer : public rclcpp::Node
{
public:
    PointTransformer()
    : Node("point_transformer"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        subscriber1_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/lop_recCamera/limb_joints_position", 10, std::bind(&PointTransformer::callback1, this, std::placeholders::_1));
        subscriber2_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/lop_recCameraR/limb_joints_position", 10, std::bind(&PointTransformer::callback2, this, std::placeholders::_1));
        subscriber3_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/lop_recCameraL/limb_joints_position", 10, std::bind(&PointTransformer::callback3, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Openpose/transformed_jointsPosition", 10);
        hand_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Openpose/transformed_handsPosition", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(30), std::bind(&PointTransformer::timer_callback, this));
    }

private:
    void callback1(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, "recCamera_color_optical_frame", 1);
    }

    void callback2(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, "recCameraR_color_optical_frame", 2);
    }

    void callback3(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        processMessage(msg, "recCameraL_color_optical_frame", 3);
    }

    void processMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg, const std::string& frame_id, int topic_number)
    {
        if (msg->data.size() == 3 && msg->data[0] == 100.0 && msg->data[1] == 100.0) {
            valid_data_received_[topic_number] = false;
            return;
        }

        valid_data_received_[topic_number] = true;
        geometry_msgs::msg::PointStamped point_in, point_out;
        std::vector<geometry_msgs::msg::Point> points;

        try {
            auto transform = tf_buffer_.lookupTransform("world", frame_id, tf2::TimePointZero);

            for (size_t i = 0; i < msg->data.size(); i += 3) {
                point_in.point.x = msg->data[i];
                point_in.point.y = msg->data[i + 1];
                point_in.point.z = msg->data[i + 2];
                tf2::doTransform(point_in, point_out, transform);

                points.emplace_back(point_out.point);
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }
        latest_points_[topic_number] = std::move(points);
    }

    void timer_callback()
    {
        if (!valid_data_received_[1] && !valid_data_received_[2] && !valid_data_received_[3]) {
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
            for (int sub = 1; sub <= 3; ++sub) {
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

        std_msgs::msg::Float32MultiArray averaged_msg;
        if (!averaged_points.empty()) {
            
            for (const auto& point : averaged_points) {
                averaged_msg.data.push_back(point.x);
                averaged_msg.data.push_back(point.y);
                averaged_msg.data.push_back(point.z);
            }
            publisher_->publish(averaged_msg);
        }

        // Create a new message for the desired values
        std_msgs::msg::Float32MultiArray hand_msg;

        // Ensure the data range [9-14] exists in averaged_msg.data before accessing them
        if (averaged_msg.data.size() >= 15 && !std::isnan(averaged_msg.data[9]) && !std::isnan(averaged_msg.data[12])) {  // The size must be at least 15 for indices 0 to 14 to be valid
            hand_msg.data = {
                averaged_msg.data[9],
                averaged_msg.data[10],
                averaged_msg.data[11],
                averaged_msg.data[12],
                averaged_msg.data[13],
                averaged_msg.data[14]
            };

            // Publish the new message using the hand_publisher_
            hand_publisher_->publish(hand_msg);

            /*
            // ************testing***************************************************************************************

            // Write hand position to a txt file
            std::ofstream outFile;
            outFile.open("Hand_openpose.txt", std::ios::app);  // Open the file in append mode
            outFile << averaged_msg.data[12] << ", " << averaged_msg.data[13] << ", " << averaged_msg.data[14] << std::endl;
            outFile.close();
            RCLCPP_INFO(get_logger(), "Hand position is written to a txt file.");
            // ***********************************************************************************************************
            */
        } 
    }


    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber1_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber2_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber3_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
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




