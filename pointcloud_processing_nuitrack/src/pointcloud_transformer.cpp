/*
// This node subscribes to the point cloud topic published by Nuitrack and transforms it to the world frame, only one camera
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>


class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Nuitrack/transformed_points", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/Nuitrack/cameraT/depth_cloud", 10, std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    }

private:

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Constructing the transformation matrix manually
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Translation
        transform(0, 3) = -0.0041363; // x translation
        transform(1, 3) = 0.2765040; // y translation
        transform(2, 3) = 1.0475700; // z translation

        // Rotation (using roll, pitch, yaw)
        double yaw = 1.53114265;
        double pitch = 0.42689265;
        double roll = 0.03388265;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
        transform.block<3, 3>(0, 0) = q.matrix();

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        // Apply filters
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(0.0, 1.5);
        pass.filter(pcl_cloud);

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-1.0, 1.0);
        pass.filter(pcl_cloud);

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 1.0);
        pass.filter(pcl_cloud);

        // Apply the transformation
        pcl::transformPointCloud(pcl_cloud, pcl_cloud, transform);

        // Convert back to ROS msg
        sensor_msgs::msg::PointCloud2 transformed;
        pcl::toROSMsg(pcl_cloud, transformed);
        transformed.header.frame_id = "world"; // Set the transformed frame ID

        publisher_->publish(transformed);
    }



  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
*/



// subscribe multi point cloud, do transformation and publish raw point cloud and voxel filtered point cloud
// use manual transformation matrix, better
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudTransformer : public rclcpp::Node
{
public:
    PointCloudTransformer() : Node("pointcloud_transformer"),
                                transform({Eigen::Matrix4f::Identity(), Eigen::Matrix4f::Identity(), Eigen::Matrix4f::Identity()})  
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/raw_final_pointscloud", 10);
        voxel_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_pointscloud", 10);

        subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Nuitrack/cameraT/depth_cloud", 10, std::bind(&PointCloudTransformer::pointCloudCallback1, this, std::placeholders::_1));
        
        subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Nuitrack/cameraR/depth_cloud", 10, std::bind(&PointCloudTransformer::pointCloudCallback2, this, std::placeholders::_1));
        
        subscription3_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/Nuitrack/cameraL/depth_cloud", 10, std::bind(&PointCloudTransformer::pointCloudCallback3, this, std::placeholders::_1));

        //timer_ = this->create_wall_timer(
        //    std::chrono::milliseconds(50),  // Adjust the rate as needed
        //    std::bind(&PointCloudTransformer::mergeAndPublish, this)
        //);

        
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

        // Constructing the transformation matrix manually, transform for top camera
        //Eigen::Matrix4f transform[0] = Eigen::Matrix4f::Identity();
        // Translation
        transform[0](0, 3) = 0.0203125; 
        transform[0](1, 3) = 0.29375;
        transform[0](2, 3) = 1.1171875; 
        // Rotation (using roll, pitch, yaw)
        double yaw = 1.579523;
        double pitch = 0.4799655;
        double roll = 0.0;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
        transform[0].block<3, 3>(0, 0) = q.matrix();

        // Constructing the transformation matrix manually, transform for right camera
        //Eigen::Matrix4f transform[1] = Eigen::Matrix4f::Identity();
        // Translation
        transform[1](0, 3) = -0.4614380; // x translation
        transform[1](1, 3) = 0.2186506; // y translation
        transform[1](2, 3) = 0.2227520; // z translation
        // Rotation (using roll, pitch, yaw)
        double yaw1 = 0.75865274;
        double pitch1 = -0.00171671;
        double roll1 = -0.01183805;
        Eigen::AngleAxisf rollAngle1(roll1, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle1(pitch1, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle1(yaw1, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q1 = yawAngle1 * pitchAngle1 * rollAngle1;
        transform[1].block<3, 3>(0, 0) = q1.matrix();

        // Constructing the transformation matrix manually, transform for left camera
        //Eigen::Matrix4f transform[0] = Eigen::Matrix4f::Identity();
        // Translation
        transform[2](0, 3) = 0.4982635; // x translation
        transform[2](1, 3) = 0.2166730; // y translation
        transform[2](2, 3) = 0.2317470; // z translation
        // Rotation (using roll, pitch, yaw)
        double yaw2 = 2.34878000;
        double pitch2 = 0.00054563;
        double roll2 = 0.00011913;
        Eigen::AngleAxisf rollAngle2(roll2, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle2(pitch2, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle2(yaw2, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q2 = yawAngle2 * pitchAngle2 * rollAngle2;
        transform[2].block<3, 3>(0, 0) = q2.matrix();
    }

private:
    void pointCloudCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        if (pcl_cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud");
            return;  // or handle the situation accordingly
        }


        // Crop the point cloud in x-axis (camera coordinate)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 1.5);
        pass.filter(pcl_cloud); 

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.6, 0.5);
        pass.filter(pcl_cloud);

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.0, 0.05);
        pass.filter(pcl_cloud);     

        //voxelFilter(pcl_cloud, pcl_cloud, 0.01f);      

        // check if the point cloud is empty, before transform
        if (pcl_cloud.empty()) {
            //RCLCPP_WARN(this->get_logger(), "Received an empty point cloud");
            if (!transformed_cloud2_.empty() || !transformed_cloud3_.empty())
            {
               mergeAndPublish();
            }
            return;  // or handle the situation accordingly
        }

        // Voxel filter
        pcl::PointCloud<pcl::PointXYZ> voxel_filtered_cloud;
        voxelFilter(pcl_cloud, voxel_filtered_cloud, 0.02f); 

        // raw point cloud
        pcl::transformPointCloud(pcl_cloud, transformed_cloud1_, transform[0]);
        // voxel filtered point cloud
        pcl::transformPointCloud(voxel_filtered_cloud, filtered_transformed_cloud1_, transform[0]);
        //RCLCPP_INFO(this->get_logger(), "transformed point cloud");
        
        // Merge and publish if all clouds are ready
        //if (!transformed_cloud2_.empty() && !transformed_cloud3_.empty())
        if (!transformed_cloud2_.empty() || !transformed_cloud3_.empty())
        {
            mergeAndPublish();
        }
    }

    void pointCloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        if (pcl_cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud");
            return;  // or handle the situation accordingly
        }


        // Crop the point cloud in x-axis (camera coordinate)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 1.2);
        pass.filter(pcl_cloud); 

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.6, 0.6);
        pass.filter(pcl_cloud);   

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.2, 1.0);
        pass.filter(pcl_cloud);    

        // check if the point cloud is empty, before transform
        if (pcl_cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud");
            return;  // or handle the situation accordingly
        }   

        // Voxel filter
        pcl::PointCloud<pcl::PointXYZ> voxel_filtered_cloud;
        voxelFilter(pcl_cloud, voxel_filtered_cloud, 0.02f); 

        // raw point cloud
        pcl::transformPointCloud(pcl_cloud, transformed_cloud2_, transform[1]);
        // voxel filtered point cloud
        pcl::transformPointCloud(voxel_filtered_cloud, filtered_transformed_cloud2_, transform[1]);
        //RCLCPP_INFO(this->get_logger(), "transformed point cloud");

    }


    void pointCloudCallback3(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        if (pcl_cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud");
            return;  // or handle the situation accordingly
        }


        // Crop the point cloud in x-axis (camera coordinate)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, 1.2);
        pass.filter(pcl_cloud); 

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-0.6, 0.6);
        pass.filter(pcl_cloud);  

        pass.setInputCloud(pcl_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.2, 1.0);
        pass.filter(pcl_cloud);     

        // check if the point cloud is empty, before transform
        if (pcl_cloud.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty point cloud");
            return;  // or handle the situation accordingly
        }   

        // Voxel filter
        pcl::PointCloud<pcl::PointXYZ> voxel_filtered_cloud;
        voxelFilter(pcl_cloud, voxel_filtered_cloud, 0.02f); 

        // raw point cloud
        pcl::transformPointCloud(pcl_cloud, transformed_cloud3_, transform[2]);
        // voxel filtered point cloud
        pcl::transformPointCloud(voxel_filtered_cloud, filtered_transformed_cloud3_, transform[2]);
        //RCLCPP_INFO(this->get_logger(), "transformed point cloud");

    }

    void mergeAndPublish()
    {
        pcl::PointCloud<pcl::PointXYZ> merged_cloud;
        merged_cloud += transformed_cloud1_;
        merged_cloud += transformed_cloud2_;
        merged_cloud += transformed_cloud3_;

        sensor_msgs::msg::PointCloud2 merged_msg;
        pcl::toROSMsg(merged_cloud, merged_msg);

        merged_msg.header.stamp = this->now();
        merged_msg.header.frame_id = "world"; 

        // publish raw merged point cloud
        publisher_->publish(merged_msg);
        

        pcl::PointCloud<pcl::PointXYZ> merged_voxel_cloud;
        merged_voxel_cloud += filtered_transformed_cloud1_;
        merged_voxel_cloud += filtered_transformed_cloud2_;
        merged_voxel_cloud += filtered_transformed_cloud3_;

        sensor_msgs::msg::PointCloud2 merged_voxel_msg;
        pcl::toROSMsg(merged_voxel_cloud, merged_voxel_msg);

        merged_voxel_msg.header.stamp = this->now();
        merged_voxel_msg.header.frame_id = "world"; 

        // publish voxel filtered merged point cloud
        voxel_publisher_->publish(merged_voxel_msg);
        

        transformed_cloud1_.clear();
        transformed_cloud2_.clear();
        transformed_cloud3_.clear();

        filtered_transformed_cloud1_.clear();
        filtered_transformed_cloud2_.clear();
        filtered_transformed_cloud3_.clear();
    }

    void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out, float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZ> vox;
        vox.setInputCloud(cloud_in.makeShared());
        vox.setLeafSize(leaf_size, leaf_size, leaf_size);
        vox.filter(cloud_out);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription3_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_publisher_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud1_;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud2_;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud3_;

    pcl::PointCloud<pcl::PointXYZ> filtered_transformed_cloud1_;
    pcl::PointCloud<pcl::PointXYZ> filtered_transformed_cloud2_;
    pcl::PointCloud<pcl::PointXYZ> filtered_transformed_cloud3_;

    std::array<Eigen::Matrix4f, 3> transform;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
