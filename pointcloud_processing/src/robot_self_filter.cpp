
// get robot joint positions and filter the point cloud of robot (robot filter is cuboid)       ******************** can run
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization_msgs/msg/marker_array.hpp>



class PointCloudFilterNode : public rclcpp::Node
{
public:
    PointCloudFilterNode()
    : Node("point_cloud_filter_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/final_Occupy_cloud", 10);
        cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/voxel_pointscloud", 10,
            std::bind(&PointCloudFilterNode::callback, this, std::placeholders::_1));
        
        points_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/Robot/joint_positions", 10,
            std::bind(&PointCloudFilterNode::pointsCallback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Robot/visualization_marker", 10);


        p1_ = Eigen::Vector3f(0, 0, 0);
        p2_ = Eigen::Vector3f(0, 0, 0);
        p3_ = Eigen::Vector3f(0, 0, 0);
        p4_ = Eigen::Vector3f(0, 0, 0);
        p5_ = Eigen::Vector3f(0, 0, 0);
        p6_ = Eigen::Vector3f(0, 0, 0);

        box_width_ = 0.06;
        box_depth_ = 0.06;
    }

private:
    void pointsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if(msg->data.size() >= 18)  
        {
            p1_ = Eigen::Vector3f(msg->data[0], msg->data[1], msg->data[2]);
            p2_ = Eigen::Vector3f(msg->data[3], msg->data[4], msg->data[5]);
            p3_ = Eigen::Vector3f(msg->data[6], msg->data[7], msg->data[8]);
            p4_ = Eigen::Vector3f(msg->data[9], msg->data[10], msg->data[11]);
            p5_ = Eigen::Vector3f(msg->data[12], msg->data[13], msg->data[14]);
            p6_ = Eigen::Vector3f(msg->data[15], msg->data[16], msg->data[17]); 
        }
        publishPointsAsMarkers();
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulator_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        pcl::fromROSMsg(*msg, *cloud);
        *accumulator_cloud = *cloud;

        std::vector<Eigen::Vector3f> points = {p1_, p2_, p3_, p4_, p5_, p6_};

        for (size_t i = 0; i < points.size() - 1; i++)
        {
            Eigen::Vector3f start_point = points[i];
            Eigen::Vector3f end_point = points[i + 1];

            Eigen::Vector3f direction = (end_point - start_point).normalized();
            float original_half_height = (end_point - start_point).norm() / 2.0;
            float half_height = original_half_height + 0.06; // Add 6 cm to the half height

            Eigen::Vector3f center = start_point + original_half_height * direction; // Note that we still calculate the center based on the original_half_height

            if (i == 1) 
            {
                box_width_ = 0.4;
                box_depth_ = 0.4;
            }

            // Calculate the rotation to align the z-axis to the orientation
            Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, 1), direction);
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.rotate(rotation.inverse());
            transform.translate(-center);

            Eigen::Vector4f min_pt(-box_width_/2, -box_depth_/2, -half_height, 1.0);
            Eigen::Vector4f max_pt(box_width_/2, box_depth_/2, half_height, 1.0);

            pcl::CropBox<pcl::PointXYZ> box_filter;
            box_filter.setTransform(transform);
            box_filter.setMin(min_pt);
            box_filter.setMax(max_pt);
            box_filter.setNegative(true);
            box_filter.setInputCloud(accumulator_cloud);
            box_filter.filter(*temp_cloud);

            accumulator_cloud->swap(*temp_cloud);
        }

        /*
        if (accumulator_cloud->points.size() == 0) {
            RCLCPP_INFO(get_logger(), "Point Cloud is empty!!!");
            return;
        }
        // Statistical Outlier Removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(accumulator_cloud);
        sor.setMeanK(30);          // For example: Consider 50 nearest neighbors.
        sor.setStddevMulThresh(1.0); // For example: Remove points that are 1 standard deviation away from the mean.
        sor.filter(*accumulator_cloud);
        */
        
        if (accumulator_cloud->points.size() == 0) {
            //RCLCPP_INFO(get_logger(), "After SOR, Point Cloud is empty!!!");
            return;
        }
        // Radius Outlier Removal filter, after test it is enough, that just use ROR filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(accumulator_cloud);
        ror.setRadiusSearch(0.2); // For example: Remove points that have less than 2 neighbors within a radius of 10cm.
        ror.setMinNeighborsInRadius(30);
        ror.filter(*accumulator_cloud);
        

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*accumulator_cloud, output);
        output.header.frame_id = msg->header.frame_id;
        //RCLCPP_INFO(get_logger(), "Point Cloud is publishing");
        publisher_->publish(output);
    }


    void publishPointsAsMarkers()
    {
        visualization_msgs::msg::MarkerArray marker_array;

        std::vector<Eigen::Vector3f> points = {p1_, p2_, p3_, p4_, p5_, p6_};

        int id = 0;
        for (const auto& pt : points)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "world"; // Replace this with appropriate frame if needed
            marker.header.stamp = this->now();
            marker.ns = "points";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = pt[0];
            marker.pose.position.y = pt[1];
            marker.pose.position.z = pt[2];
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker_array.markers.push_back(marker);
        }

        marker_publisher_->publish(marker_array);
    }



    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr points_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    Eigen::Vector3f p1_, p2_, p3_, p4_, p5_, p6_;
    float box_width_, box_depth_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFilterNode>());
    rclcpp::shutdown();
    return 0;
}