#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class Pre_grasp_pose_publisher : public rclcpp::Node
{
public:
    Pre_grasp_pose_publisher() : Node("pre_grasp_pose_publisher")
    {
        publisher_pre_grasp_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pre_grasp_pose", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(1000ms, std::bind(&Pre_grasp_pose_publisher::send_goal, this));
    }

    void send_goal()
    {
        geometry_msgs::msg::PoseStamped pre_grasp_pose_wrt_tag;


        pre_grasp_pose_wrt_tag.header.stamp = this->get_clock()->now();
        pre_grasp_pose_wrt_tag.header.frame_id = "tag36h11:1";

        pre_grasp_pose_wrt_tag.pose.position.x = -0.005;
        pre_grasp_pose_wrt_tag.pose.position.y = 0.03; //by trial and error 0.03
        pre_grasp_pose_wrt_tag.pose.position.z = 0.20;

        pre_grasp_pose_wrt_tag.pose.orientation.w = 1.0;

        geometry_msgs::msg::PoseStamped pre_grasp_pose_wrt_base;

        try
        {
            tf_buffer_->transform(pre_grasp_pose_wrt_tag, pre_grasp_pose_wrt_base, "base_link", tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "pre_grasp_pose_wrt_tag", "pre_grasp_pose_wrt_base", ex.what());
            return;
        }

        pre_grasp_pose_wrt_base.header.stamp = this->get_clock()->now();
        pre_grasp_pose_wrt_base.header.frame_id = "base_link";

        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, M_PI / 2.0);
        q.normalize();

        pre_grasp_pose_wrt_base.pose.orientation = tf2::toMsg(q);

        this->publisher_pre_grasp_pose->publish(pre_grasp_pose_wrt_base);

    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pre_grasp_pose{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Pre_grasp_pose_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}