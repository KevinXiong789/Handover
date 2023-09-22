// subscribe robot tcp point and hand position, computing distance, publish grasp flag     ****************** can run
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cmath>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

class PointProcessor : public rclcpp::Node
{
public:
  PointProcessor()
    : Node("point_processor")
  {
    /*
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/hand_pointscloud", 10,
      std::bind(&PointProcessor::cloudCallback, this, std::placeholders::_1));
    cluster_pub_ = this->create_publisher<std_msgs::msg::Bool>("/handover/cluster_detected", 1);
    */

    point1_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Robot/joint_positions", 10,
      std::bind(&PointProcessor::point1Callback, this, std::placeholders::_1));
    point2_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Openpose/transformed_handsPosition", 10,
    //point2_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/Nuitrack/transformed_handsPosition", 10,
      std::bind(&PointProcessor::point2Callback, this, std::placeholders::_1));
    bool_pub_ = this->create_publisher<std_msgs::msg::Bool>("/handover/grasp_flag", 1);

    // set timer to check distance
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&PointProcessor::checkDistance, this));

    prev_time_ = this->now();
  }

private:
    void point1Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // get robot tcp point
        if(msg->data.size() >= 3) {
        point1_[0] = msg->data[15];
        point1_[1] = msg->data[16];
        point1_[2] = msg->data[17];
        }
        //printf("robot TCP Position xyz: %.4f, %.4f, %.4f \n", point1_[0], point1_[1], point1_[2]);
    }

    void point2Callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // get right hand position
        if(msg->data.size() >= 3) {
        point2_[0] = msg->data[3];
        point2_[1] = msg->data[4];
        point2_[2] = msg->data[5];
        }
        //printf("right hand Position xyz: %.4f, %.4f, %.4f \n", point2_[0], point2_[1], point2_[2]);
    }

    void checkDistance()
    {
        double distance = std::sqrt(
            std::pow(point1_[0] - point2_[0], 2) +
            std::pow(point1_[1] - point2_[1], 2) +
            std::pow(point1_[2] - point2_[2], 2)
        );

        //printf("distance: %.4f \n", distance);

        if (distance > distance_threshold_) {
            distance_under_threshold_time_ = 0;
        } else {
            distance_under_threshold_time_ += (this->now() - prev_time_).seconds();
        }

        if (distance_under_threshold_time_ >= 2.0) {
            publishBoolMessage(true);
        } else {
            publishBoolMessage(false);
        }

        prev_time_ = this->now();
    }
/*
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if(cloud->empty()) {
            return;
        }

        // Create the KD-Tree for the clustering algorithm
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        // Using a simple clustering algorithm to detect clusters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01); // 2cm
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        printf("cluster size: %ld \n", cluster_indices.size());

        if(cluster_indices.size() == 1) {  // assuming you want exactly one cluster
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            cluster_pub_->publish(msg);
        } else {
            auto msg = std_msgs::msg::Bool();
            msg.data = false;
            cluster_pub_->publish(msg);
        }
    }
*/

    void publishBoolMessage(bool status)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = status;
        bool_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cluster_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point2_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time prev_time_;

    double point1_[3];
    double point2_[3];
    double distance_threshold_ = 0.28;
    double distance_under_threshold_time_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointProcessor>());
  rclcpp::shutdown();
  return 0;
}