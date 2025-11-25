//node that publish to /initialpose
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <array>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"



using namespace std::chrono_literals;

class Navigation : public rclcpp::Node
{
public:
  Navigation()
  : Node("navigation")
  {

    publisher_initial_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&Navigation::on_timer, this));

    initial_pose_wrt_map.header.frame_id = "map";

  }

private:
  void on_timer()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_wrt_map;

    geometry_msgs::msg::TransformStamped tf_robot_initpose_wrt_map;
   

    try {
    tf_robot_initpose_wrt_map = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } 
    catch (const tf2::TransformException & ex) 
    {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "base_link", "map", ex.what());
    return;
    }
    initial_pose_wrt_map.header.stamp = this->get_clock()->now();

    initial_pose_wrt_map.pose.position.x = tf_robot_initpose_wrt_map.transform.translation.x;
    initial_pose_wrt_map.pose.position.y = tf_robot_initpose_wrt_map.transform.translation.y;
    initial_pose_wrt_map.pose.position.z = tf_robot_initpose_wrt_map.transform.translation.z;

    initial_pose_wrt_map.pose.orientation.x = tf_robot_initpose_wrt_map.transform.rotation.x;
    initial_pose_wrt_map.pose.orientation.y = tf_robot_initpose_wrt_map.transform.rotation.y;
    initial_pose_wrt_map.pose.orientation.z = tf_robot_initpose_wrt_map.transform.rotation.z;
    initial_pose_wrt_map.pose.orientation.w = tf_robot_initpose_wrt_map.transform.rotation.w;

    //covariance matrix -- almost random initialize -- llm values
    std::array<double, 36> covariance = {};
    covariance[0] = 0.25;  //  variance x
    covariance[7] = 0.25;  // variance y
    covariance[35] = 0.0685; // varianza yaw

    initial_pose_wrt_map.pose.covariance = covariance;


    this->publisher_initial_pose->publish(initial_pose_wrt_map); //publish initial robot pose wrt map (global frame)
    timer_->cancel();
   
  } 

  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_initial_pose{nullptr};

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());
  rclcpp::shutdown();
  return 0;
}