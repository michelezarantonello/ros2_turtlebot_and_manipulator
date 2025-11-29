//node that publish to /initialpose
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <array>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"



using namespace std::chrono_literals;

class Navigation : public rclcpp::Node
{
public:
  Navigation()
  : Node("init_pose_publisher")
  {

    publisher_initial_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    subscription_amcl_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose",10,
      std::bind(&Navigation::amcl_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&Navigation::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "InitialPosePublisher started!");


  }

private:
  void on_timer()
  {

    if (amcl_pose_received_) {
      RCLCPP_INFO(this->get_logger(), "AMCL has accepted initial pose. Stopping publisher.");
      timer_->cancel();
      rclcpp::shutdown();
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_wrt_map;
   
    initial_pose_wrt_map.header.stamp = this->get_clock()->now();
    initial_pose_wrt_map.header.frame_id = "map";

    initial_pose_wrt_map.pose.pose.position.x = 0;
    initial_pose_wrt_map.pose.pose.position.y = 0;
    initial_pose_wrt_map.pose.pose.position.z = 0;

    initial_pose_wrt_map.pose.pose.orientation.x = 0;
    initial_pose_wrt_map.pose.pose.orientation.y = 0;
    initial_pose_wrt_map.pose.pose.orientation.z = 0;
    initial_pose_wrt_map.pose.pose.orientation.w = 1;

    //covariance matrix -- almost random initialize -- llm values
    std::array<double, 36> covariance = {};
    covariance[0] = 0.25;  //  variance x
    covariance[7] = 0.25;  // variance y
    covariance[14] = 0.25;  // variance z
    covariance[21] = 0.0685; // varianza yaw
    covariance[28] = 0.0685; // varianza yaw
    covariance[35] = 0.0685; // varianza yaw

    initial_pose_wrt_map.pose.covariance = covariance;


    this->publisher_initial_pose->publish(initial_pose_wrt_map); //publish initial robot pose wrt map (global frame)
    RCLCPP_INFO(this->get_logger(), "Publishing initial pose...");
   
  } 
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
      
      double xy_cov = msg->pose.covariance[0] + msg->pose.covariance[7];

      if (xy_cov < 0.5) {
        RCLCPP_INFO(this->get_logger(),
          "AMCL pose stable (cov: %.3f). Initial pose accepted!", xy_cov);
        amcl_pose_received_ = true;
      }
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  bool amcl_pose_received_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_initial_pose{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_amcl_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());
  rclcpp::shutdown();
  return 0;
}