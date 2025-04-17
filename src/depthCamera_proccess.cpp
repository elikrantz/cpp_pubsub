// importing of packages goes here
#include <chrono>
#include <functional>
#include <memory>
#include <string>	

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>


#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <depthimage_to_laserscan/DepthImageToLaserScan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


float Rock_Threshold;
float Crater_Threshold;

//const string Depth_PC2_Topic = "depth_camera/color/depth/points"
const std::string Depth_PC2_Topic = "/depth_camera/depth_camera/depth/color/points";


// creation of node for proccessing of depth camera data
class depthProccessing : public rclcpp::Node {
public:
    depthProccessing() : Node("depthProc") {
        //subscription_ = this->create_subscription<sensor_msgs::PointCloud2>(
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //subscription_ = this->create_subscription<sensor_msgs::msg::point_cloud2>(
            Depth_PC2_Topic, 10, std::bind(&depthProccessing::rotatePointCloud, this, std::placeholders::_1));

        rot_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //subscription_ = this->create_subscription<sensor_msgs::msg::point_cloud2>(
            "/temp/rotated/point_cloud2", 10, std::bind(&depthProccessing::getPointCloud2, this, std::placeholders::_1));
        
        obst_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //subscription_ = this->create_subscription<sensor_msgs::msg::point_cloud2>(
            "/obstacle/point_cloud2", 10, std::bind(&depthProccessing::pubToScan, this, std::placeholders::_1));

        // 100 commands per second (might want 1000)
        //obst_msg_ = this->create_publisher<geometry_msgs::msg::Point>("output/obstacle/points", 100);
        rot_msg_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/temp/rotated/point_cloud2",10);

        obst_msg_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle/point_cloud2",10);

        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/sensorData/depthCamData", 10);

        //rclcpp::Node::SharedPtr obst_pointCloud_msg = rclcpp::Node::make_shared("obstacle/point_cloud2_publisher");
        //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("obstacle/point_cloud_topic",10);

    }
private:
    //void getPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg) const {
    void getPointCloud2(const sensor_msgs::msg::PointCloud2 &msg) const {
    //void getPointCloud2(const sensor_msgs::msg:PointCloud2 &msg) const {
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

        // Declare message in the publishing message type
        pcl::PointCloud<pcl::PointXYZ> obst_cloud_;
        
        // TODO: Look at pcl_conversions, to convert PointCloud2 into PointCloud
        // error says something about not being able to convert PointCloud2, not sure into what

        //pcl::PointCloud2<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(msg, cloud);

        for (const auto& point : cloud) {
            
            pcl::PointXYZ obst_point = pcl::PointXYZ();
            //pcl::PointXYZ obst_point;

            //int num = 1;


            // delfaut vals crater = -0.15, rock = 0.0
            // we think its units are about 1 meter


            if (-3.0 <= point.y && point.y <= 3.0) {  // making sensor not see max
                if (point.z >= Rock_Threshold) {
                    // obstacle.x = cloud->points[i].x;
                    // obstacle.y = cloud->points[i].y;
                    // obstacle.z = cloud->points[i].y;
                    obst_point.x = point.x;
                    obst_point.y = point.y;
                    obst_point.z = point.z;
                    //obst_point.z = num;
                    //printf("\n%f.02\n",obst_point.z);
                } else if (point.z <= Crater_Threshold) {
                    // obstacle.x = cloud->points[i].x;
                    // obstacle.y = cloud->points[i].y;
                    // obstacle.z = cloud->points[i].y;
                    obst_point.x = point.x;
                    obst_point.y = point.y;
                    obst_point.z = point.z;
                    //obst_point.z = num;
                    //printf("\n%f.02\n",obst_point.z);
                }
            }
            obst_cloud_.points.push_back(obst_point);
        }

        sensor_msgs::msg::PointCloud2 obst_pc2_msg_;
        pcl::toROSMsg(obst_cloud_, obst_pc2_msg_);
        obst_pc2_msg_.header.frame_id = "odom";
        obst_pc2_msg_.header.stamp = now();
        
        obst_msg_->publish(obst_pc2_msg_);

        //std::cout <<  obst_pc2_msg_ << std::endl;
    }

    // outputs
    // sensor_msgs::ImagePtr depth_msg_;
    // sensor_msgs::CameraInfoPtr info_msg_;

    //size_t message_count_;

    void pubToScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Initialize LaserScan message
        sensor_msgs::msg::LaserScan laser_scan_msg;
        laser_scan_msg.header = msg->header;
        laser_scan_msg.header.frame_id = "odom";
        laser_scan_msg.angle_min = -M_PI / 2.0; // 90 degrees
        laser_scan_msg.angle_max = M_PI / 2.0;  // 90 degrees
        // laser_scan_msg.angle_min = M_PI * 0; // 90 degrees
        // laser_scan_msg.angle_max = M_PI * 2.0;  // 90 degrees
        laser_scan_msg.angle_increment = 0.004363;  // Resolution, ~0.25 degrees
        laser_scan_msg.time_increment = 0.0;
        laser_scan_msg.scan_time = 0.1;
        laser_scan_msg.range_min = 0.1;
        laser_scan_msg.range_max = 12.0;
        
        // Number of angles
        int num_angles = static_cast<int>((laser_scan_msg.angle_max - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment);
        laser_scan_msg.ranges.resize(num_angles, std::numeric_limits<float>::infinity());
        laser_scan_msg.intensities.resize(num_angles, 0.0);

        // Iterate over the point cloud and project points onto 2D plane
        for (const auto &point : cloud.points) {
            float angle = std::atan2(point.y, point.x);  // Compute angle from X, Y
            //float angle = point.y;
            float range = std::sqrt(point.x * point.x + point.y * point.y);  // Compute range from X, Y
            //float range = point.z;

            if (range < laser_scan_msg.range_min || range > laser_scan_msg.range_max)
            {
                continue;  // Skip points outside the range
            }

            // Find the index in the LaserScan message
            int index = static_cast<int>((angle - laser_scan_msg.angle_min) / laser_scan_msg.angle_increment);

            if (index >= 0 && index < num_angles)
            {
                // Update the range value for the angle
                laser_scan_msg.ranges[index] = std::min(laser_scan_msg.ranges[index], range);
            }
        }

        // Publish LaserScan message
        laser_pub_->publish(laser_scan_msg);
    }

    void rotatePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        
        //float theta = -M_PI / 2; // The angle of rotation in radians
        // Y rotation
        // transform_1 (0,0) = std::cos (theta);
        // transform_1 (0,2) = sin (theta);
        // transform_1 (2,0) = -sin (theta);
        // transform_1 (2,2) = std::cos (theta);
        // X rotation
        // transform_1 (1,1) = std::cos (theta);
        // transform_1 (1,2) = -sin (theta);
        // transform_1 (2,1) = sin (theta);
        // transform_1 (2,2) = std::cos (theta);
        // Z rotation
        // transform_1 (0,0) = std::cos (theta);
        // transform_1 (0,1) = -sin (theta);
        // transform_1 (1,0) = sin (theta);
        // transform_1 (1,1) = std::cos (theta);
        //float test = 0;
        float xRot = -M_PI / 2;
        float yRot = 0.0;
        float zRot = -M_PI / 2;

        // all axis rotation
        transform_1 (0,0) = std::cos(yRot) * std::cos(zRot);
        transform_1 (0,1) = sin(xRot) * sin(yRot) * std::cos(zRot) - std::cos(xRot) * sin(yRot);
        transform_1 (0,2) = std::cos(xRot) * sin(yRot) * std::cos(zRot) + sin(xRot) * sin(zRot);

        transform_1 (1,0) = std::cos(yRot) * sin(zRot);
        transform_1 (1,1) = sin(xRot) * sin(yRot) * sin(zRot) + std::cos(xRot) * std::cos(zRot);
        transform_1 (1,2) = std::cos(xRot) * sin(yRot) * sin(zRot) - sin(xRot) * std::cos(zRot);

        transform_1 (2,0) = -sin(yRot);
        transform_1 (2,1) = sin(xRot) * std::cos(yRot);
        transform_1 (2,2) = std::cos(xRot) * std::cos(yRot);

        pcl::PointCloud<pcl::PointXYZ> rot_cloud;

        pcl::transformPointCloud (cloud, rot_cloud, transform_1);

        sensor_msgs::msg::PointCloud2 rot_pc2_msg_;
        pcl::toROSMsg(rot_cloud, rot_pc2_msg_);
        rot_pc2_msg_.header.frame_id = "map";
        rot_pc2_msg_.header.stamp = now();
        
        rot_msg_->publish(rot_pc2_msg_);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obst_pc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rot_pc_sub_;
    //rclcpp::Subscription<sensor_msgs::msg::point_cloud2>::SharedPtr subscription_;

    // Declaration of the publisher attribute for diffbot
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obst_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rot_msg_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;

};

int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<depthProccessing>();
     
    node->declare_parameter<float>("rock_threshold", 0.0);
    Rock_Threshold = node->get_parameter("rock_threshold").as_double();
    node->declare_parameter<float>("crater_threshold", -0.15);
    Crater_Threshold = node->get_parameter("crater_threshold").as_double();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
