// This algorithm detects the tables from the Laser Scan data

#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class TableDetection : public rclcpp::Node
{
public:
   TableDetection()  : Node("table_detection")
   {
      RCLCPP_INFO(this->get_logger(), "Table Detection Node has been started.");

      tf_buffer_  = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
         "/scan", 10, std::bind(&TableDetection::topic_callback, this, std::placeholders::_1));
   }

private:
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
   int table_count = 0;
   // TF2 components
   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  
   void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
   {
      table_count = 0;
      double distance_threshold = 0.5; // Threshold to detect a discontinuity
      int possible_minimum = 0;
      int start_table_index = 0;
      int end_table_index = 0;
      std::vector<std::pair<double, double>> table_centers;
      
      RCLCPP_INFO(this->get_logger(), "\n"); // New line for better readability in the console

      for (size_t i = 1; i < laser_msg->ranges.size(); ++i)
      {
         // Check for discontinuity
         auto current_distance = laser_msg->ranges[i];
         auto previous_distance = laser_msg->ranges[i - 1];
         if (current_distance - previous_distance < -distance_threshold)
         {
            possible_minimum = 1; // Potential start of a table
            start_table_index = i;
         }
         else if (current_distance - previous_distance > distance_threshold && possible_minimum)
         {
            
            // Computing dimensions of the detected table
            end_table_index = i - 1;
            double angle_cluster = (end_table_index - start_table_index) * laser_msg->angle_increment; // Angle of the arc detecting table

            // Check if the arc length is eligible to be considered a table (more robust to specific cases)
            if(angle_cluster < M_PI/3.0)
            {
               table_count++;
               // Calculation of table radius
               int central_index = (start_table_index + end_table_index) / 2;
               double central_distance = laser_msg->ranges[central_index];
               double central_angle = laser_msg->angle_min + central_index * laser_msg->angle_increment;
               double table_radius = (previous_distance * tan(angle_cluster / 2.0));
            
               // Calculating coordinates of the table center w.r.t. the scanner reference frame
               double x_table = (central_distance + table_radius) * cos(central_angle);
               double y_table = (central_distance + table_radius) * sin(central_angle);

               // Saving table center coordinates
               table_centers.push_back(std::make_pair(x_table, y_table));
            }
            possible_minimum = 0; // Reset for next detection
         }
      }
      RCLCPP_INFO(this->get_logger(), "Detected %d tables.", table_count);
      
      for(size_t j = 0; j < table_centers.size(); j++)
      {
         // Setting Pose of tables in the scan frame
         geometry_msgs::msg::PoseStamped table_pose;
         table_pose.header.stamp = this->get_clock()->now();
         table_pose.header.frame_id = laser_msg->header.frame_id; // frame of the laser
         table_pose.pose.position.x = table_centers[j].first;
         table_pose.pose.position.y = table_centers[j].second;

         // Transforming table pose to the "odom" frame
         try {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform("odom", laser_msg->header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped pose_odom;
            tf2::doTransform(table_pose, pose_odom, transform_stamped);
            pose_odom.pose.position.z = 0.0; // Assuming tables are on the ground plane      
            RCLCPP_INFO(this->get_logger(), "Table %ld position in %s frame: x = %.2f, y = %.2f, z = 0", j+1, pose_odom.header.frame_id.c_str(), pose_odom.pose.position.x, pose_odom.pose.position.y);
         } catch (tf2::TransformException &ex) {
         RCLCPP_WARN(get_logger(), "TF failed: %s", ex.what());
         }
      }
   }
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<TableDetection>());
   rclcpp::shutdown();
   return 0;
}