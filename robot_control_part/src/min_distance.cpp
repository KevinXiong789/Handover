/*
// Min_distanc between Robot (6 Joints and their connections) and Point cloud           **********************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float64.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/processed_point_cloud", 10, std::bind(&JointPoseListener::point_cloud_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pointcloud/minimum_distance", 10);
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Check if the point cloud is empty
        if(cloud->points.empty()) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = 10.0; // Setting distance to 10 when cloud is empty
            RCLCPP_INFO(this->get_logger(), "Point cloud is empty. Min_distance is set to %f", distance_msg.data);
            distance_pub_->publish(distance_msg);
            return;  // Exit the callback
        }

        auto kinematic_state = move_group_interface_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("ur_manipulator");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        double min_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < joint_names.size() - 1; ++i)
        {
            const moveit::core::JointModel* joint_model1 = robot_model_->getJointModel(joint_names[i]);
            const moveit::core::JointModel* joint_model2 = robot_model_->getJointModel(joint_names[i+1]);
            if (joint_model1 && joint_model2)
            {
                const std::string& child_link_name1 = joint_model1->getChildLinkModel()->getName();
                const std::string& child_link_name2 = joint_model2->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state1 = kinematic_state->getGlobalLinkTransform(child_link_name1);
                Eigen::Isometry3d link_state2 = kinematic_state->getGlobalLinkTransform(child_link_name2);
                Eigen::Vector3d link_position1 = link_state1.translation();
                Eigen::Vector3d link_position2 = link_state2.translation();

                for (const auto& point : cloud->points)
                {
                    Eigen::Vector3d point_position(point.x, point.y, point.z);
                    double distance = distancePointSegment(point_position, link_position1, link_position2);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                }
            }
        }

        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    double distancePointSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& segStart, const Eigen::Vector3d& segEnd)
    {
        Eigen::Vector3d diff = point - segStart;
        Eigen::Vector3d dir = segEnd - segStart;
        double t = diff.dot(dir);
        if (t <= 0.0)
        {
            // point is nearest to segStart
            t = 0.0;
        }
        else
        {
            double sqlen = dir.squaredNorm();  // segment length squared
            if (t >= sqlen)
            {
                // point is nearest to segEnd
                t = 1.0;
                diff -= dir;
            }
            else
            {
                // point is nearest to middle of segment
                t /= sqlen;
                diff -= dir * t;
            }
        }
        return diff.norm();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPoseListener>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/



/*
// Min_distanc between Robot (6 Joints and their connections) and Human Skeleton Joints Points       ************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/float64.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <Eigen/Dense>
#include <std_msgs/msg/float32_multi_array.hpp>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        joint_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Openpose/transformed_jointsPosition", 10, std::bind(&JointPoseListener::joint_position_callback, this, std::placeholders::_1));
            //"Nuitrack/transformed_jointsPosition", 10, std::bind(&JointPoseListener::joint_position_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Jointpoints/minimum_distance", 10);

    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader_->getModel();
    }

private:
    void joint_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::vector<Eigen::Vector3d> points;
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            
            float x = msg->data[i];
            float y = msg->data[i+1];
            float z = msg->data[i+2];
            
            if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                points.push_back(Eigen::Vector3d(x, y, z));
                
            }
        }

        if (msg->data[0] == 100) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = 10.0;  // Setting distance to 10 when all points are nan
            RCLCPP_INFO(this->get_logger(), "All joint points are NaN. Min_distance is set to %f", distance_msg.data);
            distance_pub_->publish(distance_msg);
            return;  // Exit the callback
        }

        auto kinematic_state = move_group_interface_->getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup("ur_manipulator");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        RCLCPP_INFO(this->get_logger(), "every thing are good until here");
        //double min_distance = std::numeric_limits<double>::max();
        double min_distance = 10.0;
        
        for (size_t i = 0; i < joint_names.size() - 1; ++i)
        {
            const moveit::core::JointModel* joint_model1 = robot_model_->getJointModel(joint_names[i]);
            const moveit::core::JointModel* joint_model2 = robot_model_->getJointModel(joint_names[i+1]);
            if (joint_model1 && joint_model2)
            {
                const std::string& child_link_name1 = joint_model1->getChildLinkModel()->getName();
                const std::string& child_link_name2 = joint_model2->getChildLinkModel()->getName();
                Eigen::Isometry3d link_state1 = kinematic_state->getGlobalLinkTransform(child_link_name1);
                Eigen::Isometry3d link_state2 = kinematic_state->getGlobalLinkTransform(child_link_name2);
                Eigen::Vector3d link_position1 = link_state1.translation();
                Eigen::Vector3d link_position2 = link_state2.translation();

                for (const auto& point : points)
                {
                    double distance = distancePointSegment(point, link_position1, link_position2);
                    if (distance < min_distance)
                    {
                        min_distance = distance;
                    }
                }
            }
        }
        
        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    double distancePointSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& segStart, const Eigen::Vector3d& segEnd)
    {
        Eigen::Vector3d diff = point - segStart;
        Eigen::Vector3d dir = segEnd - segStart;
        if ((segEnd - segStart).isZero()) {
            return (point - segStart).norm();
        }

        double t = diff.dot(dir);
        if (t <= 0.0)
        {
            // point is nearest to segStart
            t = 0.0;
        }
        else
        {
            double sqlen = dir.squaredNorm();  // segment length squared
            if (t >= sqlen)
            {
                // point is nearest to segEnd
                t = 1.0;
                diff -= dir;
            }
            else
            {
                // point is nearest to middle of segment
                t /= sqlen;
                diff -= dir * t;
            }
        }
        return diff.norm();
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPoseListener>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/


/*
// just get the position of roboter joints  *********************************** can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

class RobotLinkPositionListener : public rclcpp::Node
{
public:
    RobotLinkPositionListener()
    : Node("robot_link_position_listener")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RobotLinkPositionListener::timer_callback, this));
    }

    void init()
    {
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur_manipulator");
    }

private:
    void timer_callback()
    {
        auto current_state = move_group_interface_->getCurrentState();
        const std::vector<std::string>& link_names = move_group_interface_->getLinkNames();
        
        for(const auto& link_name : link_names)
        {
            const Eigen::Isometry3d& link_state = current_state->getGlobalLinkTransform(link_name);
            Eigen::Vector3d link_position = link_state.translation();

            RCLCPP_INFO(this->get_logger(), "%s position: x: %f, y: %f, z: %f", 
                        link_name.c_str(), 
                        link_position.x(), 
                        link_position.y(), 
                        link_position.z());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
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
*/


/*
// Min_distanc between Robot (6 Joints and their connections) and Human Skeleton Joints Points (from 3 cameras averaged)   ************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <std_msgs/msg/float32_multi_array.hpp>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        joint_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Openpose/transformed_jointsPosition", 10, std::bind(&JointPoseListener::joint_position_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/Jointpoints/minimum_distance", 10);
        link_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Robot/joint_positions", 10, std::bind(&JointPoseListener::link_position_callback, this, std::placeholders::_1));
    }

private:

    void link_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        link_positions_.clear();

        // Ensure that the data size is a multiple of 3 (x, y, z)
        if (msg->data.size() % 3 == 0) {
            for (size_t i = 0; i < msg->data.size(); i += 3) {
                float x = msg->data[i];
                float y = msg->data[i+1];
                float z = msg->data[i+2];
                
                if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                    link_positions_.push_back(Eigen::Vector3d(x, y, z));
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received data size is not a multiple of 3. Skipping this message.");
        }
    }


    void joint_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data[0] == 100) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = 10.0;  // Setting distance to 10 when all points are nan
            RCLCPP_INFO(this->get_logger(), "All joint points are NaN. Min_distance is set to %f", distance_msg.data);
            distance_pub_->publish(distance_msg);
            return;  // Exit the callback
        }
        
        std::vector<Eigen::Vector3d> points;
        for (size_t i = 0; i < msg->data.size(); i += 3) {
            
            float x = msg->data[i];
            float y = msg->data[i+1];
            float z = msg->data[i+2];
            
            if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                points.push_back(Eigen::Vector3d(x, y, z));
            }
        }
        
        double min_distance = 10.0;
        
        for (size_t i = 0; i < link_positions_.size() - 1; ++i)
        {
            Eigen::Vector3d link_position1 = link_positions_[i];
            Eigen::Vector3d link_position2 = link_positions_[i+1];
            
            for (const auto& point : points)
            {
                double distance = distancePointSegment(point, link_position1, link_position2);
                if (distance < min_distance)
                {
                    min_distance = distance;
                }
            }
            
        }
        
        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    double distancePointSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& segStart, const Eigen::Vector3d& segEnd)
    {
        Eigen::Vector3d diff = point - segStart;
        Eigen::Vector3d dir = segEnd - segStart;
        if ((segEnd - segStart).isZero()) {
            return (point - segStart).norm();
        }

        double t = diff.dot(dir);
        if (t <= 0.0)
        {
            t = 0.0;
        }
        else
        {
            double sqlen = dir.squaredNorm();
            if (t >= sqlen)
            {
                t = 1.0;
                diff -= dir;
            }
            else
            {
                t /= sqlen;
                diff -= dir * t;
            }
        }
        return diff.norm();
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr link_position_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    std::vector<Eigen::Vector3d> link_positions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPoseListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
*/



// Min_distanc between Robot (6 Joints and their connections) and Occupy_cloud        **********************************************can run
#include <memory>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>

class JointPoseListener : public rclcpp::Node
{
public:
    JointPoseListener()
    : Node("joint_pose_listener")
    {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/final_Occupy_cloud", 10, std::bind(&JointPoseListener::point_cloud_callback, this, std::placeholders::_1));
        distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pointcloud/minimum_distance", 10);
        link_position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Robot/joint_positions", 10, std::bind(&JointPoseListener::link_position_callback, this, std::placeholders::_1));
    }

private:
    void link_position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        link_positions_.clear();

        // Ensure that the data size is a multiple of 3 (x, y, z)
        if (msg->data.size() % 3 == 0) {
            for (size_t i = 0; i < msg->data.size(); i += 3) {
                float x = msg->data[i];
                float y = msg->data[i+1];
                float z = msg->data[i+2];
                
                if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z)) {
                    link_positions_.push_back(Eigen::Vector3d(x, y, z));
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received data size is not a multiple of 3. Skipping this message.");
        }
    }
    
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Check if the point cloud is empty
        if(cloud->points.empty()) {
            std_msgs::msg::Float64 distance_msg;
            distance_msg.data = 10.0; // Setting distance to 10 when cloud is empty
            RCLCPP_INFO(this->get_logger(), "Point cloud is empty. Min_distance is set to %f", distance_msg.data);
            distance_pub_->publish(distance_msg);
            return;  // Exit the callback
        }

        double min_distance = 10.0;
        
        for (size_t i = 0; i < link_positions_.size() - 1; ++i)
        {
            Eigen::Vector3d link_position1 = link_positions_[i];
            Eigen::Vector3d link_position2 = link_positions_[i+1];

            for (const auto& point : cloud->points)
            {
                Eigen::Vector3d point_position(point.x, point.y, point.z);
                double distance = distancePointSegment(point_position, link_position1, link_position2);
                if (distance < min_distance)
                {
                    min_distance = distance;
                }
            }
            
        }
        
        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = min_distance;
        RCLCPP_INFO(this->get_logger(), "Min_distance is %f", min_distance);
        distance_pub_->publish(distance_msg);
    }

    double distancePointSegment(const Eigen::Vector3d& point, const Eigen::Vector3d& segStart, const Eigen::Vector3d& segEnd)
    {
        Eigen::Vector3d diff = point - segStart;
        Eigen::Vector3d dir = segEnd - segStart;
        double t = diff.dot(dir);
        if (t <= 0.0)
        {
            // point is nearest to segStart
            t = 0.0;
        }
        else
        {
            double sqlen = dir.squaredNorm();  // segment length squared
            if (t >= sqlen)
            {
                // point is nearest to segEnd
                t = 1.0;
                diff -= dir;
            }
            else
            {
                // point is nearest to middle of segment
                t /= sqlen;
                diff -= dir * t;
            }
        }
        return diff.norm();
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr link_position_sub_;
    std::vector<Eigen::Vector3d> link_positions_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPoseListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

