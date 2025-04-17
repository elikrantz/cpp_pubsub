
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>

//#include "sensor_msgs/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include <cmath>
#include <vector>

using std::placeholders::_1;



sensor_msgs::msg::PointCloud2 laser_scan_to_pointcloud2(const sensor_msgs::msg::LaserScan::SharedPtr &scan_msg)
{
    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header = scan_msg->header;
    cloud_msg.height = 1;  // Single row of points
    cloud_msg.width = scan_msg->ranges.size();  // Number of points in the scan_msg

    // Define the fields for the PointCloud2 message
    sensor_msgs::msg::PointField x_field, y_field, z_field;
    x_field.name = "x";
    x_field.offset = 0;
    x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x_field.count = 1;

    y_field.name = "y";
    y_field.offset = 4;
    y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    y_field.count = 1;

    z_field.name = "z";
    z_field.offset = 8;
    z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    z_field.count = 1;

    cloud_msg.fields.push_back(x_field);
    cloud_msg.fields.push_back(y_field);
    cloud_msg.fields.push_back(z_field);

    cloud_msg.is_bigendian = false;  // Data is in little-endian
    cloud_msg.point_step = 12;       // Size of a point in bytes (3 * 4 bytes for x, y, z)
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
    
    // Use an iterator to fill the data
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    // Convert each LaserScan reading to a 3D point (x, y, z)
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
      float range = scan_msg->ranges[i];

      if (range >= scan_msg->range_min && range <= scan_msg->range_max) 
      {
        // Convert polar coordinates (range, angle) to Cartesian (x, y)
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        *iter_x = range * cos(angle);
        *iter_y = range * sin(angle);
        *iter_z = 0.0f;  // Assuming 2D scan_msg, so z is 0
      }
      else
      {
        // Mark points out of range as NaN
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
      }

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    return cloud_msg;
}


sensor_msgs::msg::PointCloud2 merge_pointclouds(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud1,
                                               const sensor_msgs::msg::PointCloud2::SharedPtr &cloud2)
{
    pcl::PointCloud<pcl::PointXYZ>  mPtrPointCloud; // combined pointcloud
    // Cloud 1
    pcl::PointCloud<pcl::PointXYZ> pcl1_transformed;
    pcl::fromROSMsg(*cloud1, pcl1_transformed);
    // Eigen::Transform<Scalar, 3, Eigen::Affine> recieved1_transformation_mat(recieved1.sensor_origin_ * recieved1.sensor_orientation_);
    // pcl::transformPointCloud(recieved1, pcl1_transformed, recieved1_transformation_mat);
    // Cloud 2
    pcl::PointCloud<pcl::PointXYZ> pcl2_transformed;
    pcl::fromROSMsg(*cloud2, pcl2_transformed);
    // Eigen::Transform<Scalar, 3, Eigen::Affine> recieved2_transformation_mat(recieved2.sensor_origin_ * recieved2.sensor_orientation_);
    // pcl::transformPointCloud(recieved2, pcl2_transformed, recieved2_transformation_mat);
    mPtrPointCloud += pcl1_transformed;
    mPtrPointCloud += pcl2_transformed;

    sensor_msgs::msg::PointCloud2 combined_cloud;
    pcl::toROSMsg(mPtrPointCloud, combined_cloud);
    // combined_cloud.header.frame_id = "odom";
    // combined_cloud.header.stamp = now();

    return combined_cloud;
}


class PointCloudCombiner : public rclcpp::Node
{
public:
    PointCloudCombiner()
    : Node("pointcloud_combiner")
    {
        // Create subscriptions to the LaserScan and PointCloud2 topics
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/sensorData/lidarData", 10, std::bind(&PointCloudCombiner::laser_scan_callback, this, std::placeholders::_1));
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacle/point_cloud2", 10, std::bind(&PointCloudCombiner::pointcloud_callback, this, std::placeholders::_1));

        combined_pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/combined_pointcloud", 10, std::bind(&PointCloudCombiner::pubToScan, this, std::placeholders::_1));

        
        laserScan_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/laserScan_pointcloud", 10);

        // Create a publisher for the combined PointCloud2
        combined_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/combined_pointcloud", 10);

        combined_laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan", 10);
    }

private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Convert the LaserScan message to a PointCloud2 message
        sensor_msgs::msg::PointCloud2 laser_pc = laser_scan_to_pointcloud2(msg);

        laser_pc.header.frame_id = "odom";
        // combined_pc.header.stamp = now();
        // Publish the combined point cloud
        laserScan_pc_pub_->publish(laser_pc);

        // Now you can combine with another PointCloud2 message if needed 
        // Assuming you have a stored PointCloud2 message to merge with
        if (last_pointcloud_)
        {
            // Create a shared pointer from laser_pc
            auto laser_pc_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(laser_pc);
            
            // Merge the PointCloud2 messages
            sensor_msgs::msg::PointCloud2 combined_pc = merge_pointclouds(laser_pc_ptr, last_pointcloud_);
            
            combined_pc.header.frame_id = "odom";
            // combined_pc.header.stamp = now();
            // Publish the combined point cloud
            combined_pc_pub_->publish(combined_pc);
        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Store the received PointCloud2 message for merging
        last_pointcloud_ = msg;
    }

    void pubToScan(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Initialize LaserScan message
        sensor_msgs::msg::LaserScan laser_scan_msg;
        laser_scan_msg.header = msg->header;
        laser_scan_msg.header.frame_id = "odom";
        laser_scan_msg.angle_min = -M_PI; // 90 degrees
        laser_scan_msg.angle_max = M_PI;  // 90 degrees
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
        combined_laser_pub_->publish(laser_scan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserScan_pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr combined_laser_pub_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_pointcloud_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudCombiner>());
    rclcpp::shutdown();
    return 0;
}