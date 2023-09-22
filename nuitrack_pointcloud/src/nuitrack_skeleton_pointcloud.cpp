// Use Nuitrack to get skeleton key point and convert depth frame to pointcloud2, publish right hand position and pointcloud
#include <iostream>
#include <nuitrack/Nuitrack.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>


using namespace tdv::nuitrack;

class NuitrackApp : public rclcpp::Node
{
public:
    NuitrackApp() : Node("nuitrack_app")
    {
        joints_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Nuitrack/cameraT/joints_position", 10);
        hand_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/Nuitrack/right_hand", 10);
        // Initialize publishers for depth image and point cloud
        depth_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Nuitrack/cameraT/depth_cloud", 10);
        marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/Nuitrack/cameraT/joint_markers", 10);
 
        Nuitrack::init("");
        auto devices = Nuitrack::getDeviceList();
        auto selectedDevice = devices[0]; // 0 is top, 1 is right, 2 is left
        RCLCPP_INFO(get_logger(), "Selected device: %s with serial number: %s",
            selectedDevice->getInfo(tdv::nuitrack::device::DeviceInfoType::DEVICE_NAME).c_str(),
            selectedDevice->getInfo(tdv::nuitrack::device::DeviceInfoType::SERIAL_NUMBER).c_str());

        Nuitrack::setDevice(selectedDevice);

        // Create Skeleton Tracker
        skeletonTracker_ = SkeletonTracker::create();
        skeletonTracker_->connectOnUpdate(std::bind(&NuitrackApp::onSkeletonUpdate, this, std::placeholders::_1));

        // Create Depth Sensor
        depthSensor_ = DepthSensor::create();
        depthSensor_->connectOnNewFrame(std::bind(&NuitrackApp::onNewDepthFrame, this, std::placeholders::_1));

  
        // Start Nuitrack
        Nuitrack::run();
        while (rclcpp::ok()) { 
            Nuitrack::waitUpdate(skeletonTracker_);
            //Nuitrack::waitUpdate(depthSensor_);
        }
        Nuitrack::release();
    }

private:
    void onSkeletonUpdate(SkeletonData::Ptr skeletonData)
    {
        // Constructing the transformation matrix
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        // Translation
        transform(0, 3) = 0.0203125; 
        transform(1, 3) = 0.29375;
        transform(2, 3) = 1.1171875; 

        // Rotation (using roll, pitch, yaw)
        double yaw = 1.579523;
        double pitch = 0.4799655;
        double roll = 0.0;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = yawAngle * pitchAngle * rollAngle;
        transform.block<3, 3>(0, 0) = q.matrix();

        const std::vector<Skeleton> skeletons = skeletonData->getSkeletons();
        if (skeletons.empty()) {
            // Publish the specific message when skeletons is empty
            std_msgs::msg::Float32MultiArray empty_message;
            empty_message.data = {100.0, 100.0, 100.0};
            joints_publisher_->publish(empty_message);
            //RCLCPP_INFO(this->get_logger(), "Nobody is in the area.");
            return;  // Return early to avoid the remaining logic
        }

        
        for (const Skeleton& skeleton : skeletons) {
            // Get Right Hand joint
            const Joint& rightHand = skeleton.joints[JOINT_RIGHT_HAND];
            const Joint& rightElbow = skeleton.joints[JOINT_RIGHT_ELBOW];
            
            Eigen::Vector4f rightHandPoint(rightHand.real.z / 1000.0, -rightHand.real.x / 1000.0, rightHand.real.y / 1000.0, 1.0);
            Eigen::Vector4f rightElbowPoint(rightElbow.real.z / 1000.0, -rightElbow.real.x / 1000.0, rightElbow.real.y / 1000.0, 1.0);

            // Apply transformation
            rightHandPoint = transform * rightHandPoint;
            rightElbowPoint = transform * rightElbowPoint;

            std_msgs::msg::Float32MultiArray joint_coordinates;
            joint_coordinates.data = {
                rightHandPoint(0), rightHandPoint(1), rightHandPoint(2),
                rightElbowPoint(0), rightElbowPoint(1), rightElbowPoint(2)
            };
            hand_publisher_->publish(joint_coordinates);

            std_msgs::msg::Float32MultiArray all_joints_coordinates;

            const Joint& head = skeleton.joints[JOINT_HEAD];
            const Joint& leftShoulder = skeleton.joints[JOINT_LEFT_SHOULDER];
            const Joint& rightShoulder = skeleton.joints[JOINT_RIGHT_SHOULDER];
            const Joint& leftElbow = skeleton.joints[JOINT_LEFT_ELBOW];
            const Joint& leftHand = skeleton.joints[JOINT_LEFT_HAND];

            // Transform the coordinates
            Eigen::Vector4f headPoint(head.real.z / 1000.0, -head.real.x / 1000.0, head.real.y / 1000.0, 1.0);
            Eigen::Vector4f leftShoulderPoint(leftShoulder.real.z / 1000.0, -leftShoulder.real.x / 1000.0, leftShoulder.real.y / 1000.0, 1.0);
            Eigen::Vector4f rightShoulderPoint(rightShoulder.real.z / 1000.0, -rightShoulder.real.x / 1000.0, rightShoulder.real.y / 1000.0, 1.0);
            Eigen::Vector4f leftElbowPoint(leftElbow.real.z / 1000.0, -leftElbow.real.x / 1000.0, leftElbow.real.y / 1000.0, 1.0);
            Eigen::Vector4f leftHandPoint(leftHand.real.z / 1000.0, -leftHand.real.x / 1000.0, leftHand.real.y / 1000.0, 1.0);

            // Apply transformation
            headPoint = transform * headPoint;
            leftShoulderPoint = transform * leftShoulderPoint;
            rightShoulderPoint = transform * rightShoulderPoint;
            leftElbowPoint = transform * leftElbowPoint;
            leftHandPoint = transform * leftHandPoint;

            all_joints_coordinates.data = {
                headPoint(0), headPoint(1), headPoint(2),
                leftShoulderPoint(0), leftShoulderPoint(1), leftShoulderPoint(2),
                rightShoulderPoint(0), rightShoulderPoint(1), rightShoulderPoint(2),
                leftElbowPoint(0), leftElbowPoint(1), leftElbowPoint(2),
                leftHandPoint(0), leftHandPoint(1), leftHandPoint(2),
                rightElbowPoint(0), rightElbowPoint(1), rightElbowPoint(2),
                rightHandPoint(0), rightHandPoint(1), rightHandPoint(2)
            };

            joints_publisher_->publish(all_joints_coordinates);

            
        }
        publishJointMarkers(skeletonData->getSkeletons());
    }


    void publishJointMarkers(const std::vector<Skeleton>& skeletons) {
        visualization_msgs::msg::MarkerArray marker_array;

        int marker_id = 0; // Unique ID for each marker

        for (const Skeleton& skeleton : skeletons) {
            for (const Joint& joint : skeleton.joints) {
                visualization_msgs::msg::Marker marker;
                
                marker.header.frame_id = "cameraT_link"; 
                marker.header.stamp = this->now();
                marker.ns = "joints";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                // Set the marker position to the joint position
                marker.pose.position.x = joint.real.z / 1000.0;
                marker.pose.position.y = -joint.real.x / 1000.0;
                marker.pose.position.z = joint.real.y / 1000.0;
                marker.pose.orientation.w = 1.0;  // No rotation

                if (marker.pose.position.x == 0.0 && marker.pose.position.y == 0.0 && marker.pose.position.z == 0.0) {
                    continue;  // Skip the joint if the position is 0,0,0
                }

                marker.scale.x = 0.05;  // Set the size of the marker
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                marker.color.r = 0.0; // Set the color
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0; // Don't forget to set the alpha!

                marker.lifetime = rclcpp::Duration::from_seconds(0.1); // Duration the marker will be shown

                marker_array.markers.push_back(marker);
            }
        }

        marker_array_publisher_->publish(marker_array);
    }

    void onNewDepthFrame(DepthFrame::Ptr frame)
    {
        int _width = frame->getCols(); 
        int _height = frame->getRows();
        const uint16_t* depthPtr = frame->getData();
        
        int step_size = 2; 

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "cameraT_link";
        cloud_msg.width = _width / step_size;
        cloud_msg.height = _height / step_size;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;
        cloud_msg.point_step = sizeof(float) * 3;  // XYZ
        cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
        cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

        // Define the fields in the PointCloud2 message
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x"; cloud_msg.fields[0].offset = 0; cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y"; cloud_msg.fields[1].offset = 4; cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z"; cloud_msg.fields[2].offset = 8; cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[2].count = 1;

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

        const uint16_t* row_ptr = depthPtr;
        for (int row = 0; row < _height; row += step_size)
        {
            const uint16_t* col_ptr = row_ptr;
            for (int col = 0; col < _width; col += step_size)
            {
                uint16_t fulldepthValue = *col_ptr;

                Vector3 cloud_point = depthSensor_->convertProjToRealCoords(col, row, fulldepthValue);
                float X_World = cloud_point.x / 1000.0; 
                float Y_World = cloud_point.y / 1000.0;
                float Z_World = cloud_point.z / 1000.0; 

                *out_x = Z_World;
                *out_y = -X_World;
                *out_z = Y_World;

                ++out_x;
                ++out_y;
                ++out_z;
                col_ptr += step_size;
            }
            row_ptr += _width * step_size;
        }

        depth_cloud_pub_->publish(cloud_msg);
    }


private:
    SkeletonTracker::Ptr skeletonTracker_;
    DepthSensor::Ptr depthSensor_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nuitrack_app = std::make_shared<NuitrackApp>();
    rclcpp::spin(nuitrack_app);
    rclcpp::shutdown();
    return 0;
}






/*
void onNewDepthFrame(DepthFrame::Ptr frame)
    {
        int _width = frame->getCols(); 
        int _height = frame->getRows();
        const uint16_t* depthPtr = frame->getData();
        
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "cameraR_link";
        cloud_msg.width = _width;
        cloud_msg.height = _height;
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = false;
        cloud_msg.point_step = sizeof(float) * 3;  // XYZ
        cloud_msg.row_step = cloud_msg.point_step * _width;

        // Define the fields in the PointCloud2 message
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x"; cloud_msg.fields[0].offset = 0; cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y"; cloud_msg.fields[1].offset = 4; cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z"; cloud_msg.fields[2].offset = 8; cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32; cloud_msg.fields[2].count = 1;
        cloud_msg.data.resize(cloud_msg.row_step * _height);

        sensor_msgs::PointCloud2Modifier cloud_mod(cloud_msg);
        cloud_mod.resize(_width * _height);
        
        sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg, "z");

        for (int row = 0; row < _height; ++row)
        {
            for (int col = 0; col < _width; ++col)
            {
                uint16_t fulldepthValue = *(depthPtr+ col);
                //uint16_t depthValue = *(depthPtr+ col) >> 5;
                
                Vector3 cloud_point = depthSensor_->convertProjToRealCoords(col, row, fulldepthValue );
                float X_World = cloud_point.x / 1000.0; 
                float Y_World = cloud_point.y / 1000.0;
                float Z_World = cloud_point.z / 1000.0; 
                
                *out_x = Z_World;
                *out_y = -X_World;
                *out_z = Y_World; 

                ++out_x;
                ++out_y;
                ++out_z;
            }
            depthPtr += _width; 
        }

        depth_cloud_pub_->publish(cloud_msg);
    }
    */