// This algorithm detects the tables from the Laser Scan data

#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class TableDetection : public rclcpp::Node
{
public:
   TableDetection()  : Node("table_detection")
   {
      RCLCPP_INFO(this->get_logger(), "Table Detection Node has been started.");
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
         "/scan", 10, std::bind(&TableDetection::topic_callback, this, std::placeholders::_1));
   }

private:
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
   int table_count = 0;
  
   void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg)
   {

      table_count = 0;
      double distance_threshold = 0.5; // Threshold to detect a discontinuity
      int possible_minimum = 0;
      int start_table_index = 0;
      int end_table_index = 0;
      
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
            // Confirmed end of a table
            table_count++;

            // Computing central point of the detected table
            end_table_index = i - 1;
            double angle_cluster = (end_table_index - start_table_index) * laser_msg->angle_increment; // Angle of the arc detecting table
            int central_index = (start_table_index + end_table_index) / 2;
            double central_distance = laser_msg->ranges[central_index];
            double central_angle = laser_msg->angle_min + central_index * laser_msg->angle_increment;

            // Estimation of table radius
            double table_radius = 1.5 * angle_cluster/M_PI * std::pow(central_distance,2.0);
            RCLCPP_INFO(this->get_logger(), "Table %d: Central Angle: %.2f deg, Central Distance: %.2f m, Estimated Radius: %.2f m",
                        table_count, central_angle*180/M_PI, central_distance, table_radius);
            
            // Calculating coordinates of the table center w.r.t. the scanner reference frame
            double x_table = (central_distance + table_radius) * cos(central_angle);
            double y_table = (central_distance + table_radius) * sin(central_angle);
            RCLCPP_INFO(this->get_logger(), "Center Coordinates: (%.2f, %.2f) m", x_table, y_table);
            possible_minimum = 0; // Reset for next detection
         }
         
      }
      RCLCPP_INFO(this->get_logger(), "Detected %d tables.", table_count);
   }
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<TableDetection>());
   rclcpp::shutdown();
   return 0;
}
