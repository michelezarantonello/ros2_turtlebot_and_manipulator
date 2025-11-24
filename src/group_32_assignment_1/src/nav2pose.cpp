#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(this, "navigate_to_pose");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        auto timer_callback_lambda = [this](){ return this->send_goal(); };
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback_lambda);

        goal_pose_wrt_map.header.frame_id = "map";
    }

    void send_goal()
    {
        geometry_msgs::msg::TransformStamped tf_apriltag1_wrt_map;
        geometry_msgs::msg::TransformStamped tf_apriltag2_wrt_map;

        geometry_msgs::msg::PoseStamped apriltag1_pose_wrt_map;
        geometry_msgs::msg::PoseStamped apriltag2_pose_wrt_map;

        try{tf_apriltag1_wrt_map = tf_buffer_->lookupTransform("map", "tag_0", tf2::TimePointZero);} 
        catch(const tf2::TransformException & ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "tag_0", "map", ex.what());
        return;
        }
        apriltag1_pose_wrt_map.header.stamp = this->get_clock()->now();
        apriltag1_pose_wrt_map.header.frame_id = "map";

        apriltag1_pose_wrt_map.pose.position.x = tf_apriltag1_wrt_map.transform.translation.x;
        apriltag1_pose_wrt_map.pose.position.y = tf_apriltag1_wrt_map.transform.translation.y;
        apriltag1_pose_wrt_map.pose.position.z = 0.0; //projection on floor plane   

        apriltag1_pose_wrt_map.pose.orientation.x = tf_apriltag1_wrt_map.transform.rotation.x;
        apriltag1_pose_wrt_map.pose.orientation.y = tf_apriltag1_wrt_map.transform.rotation.y;
        apriltag1_pose_wrt_map.pose.orientation.z = tf_apriltag1_wrt_map.transform.rotation.z;
        apriltag1_pose_wrt_map.pose.orientation.w = tf_apriltag1_wrt_map.transform.rotation.w;

        
        try{tf_apriltag1_wrt_map = tf_buffer_->lookupTransform("map", "tag_0", tf2::TimePointZero);} 
        catch(const tf2::TransformException & ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "tag_0", "map", ex.what());
        return;
        }
        apriltag1_pose_wrt_map.header.stamp = this->get_clock()->now();
        apriltag1_pose_wrt_map.header.frame_id = "map";

        apriltag1_pose_wrt_map.pose.position.x = tf_apriltag1_wrt_map.transform.translation.x;
        apriltag1_pose_wrt_map.pose.position.y = tf_apriltag1_wrt_map.transform.translation.y;
        apriltag1_pose_wrt_map.pose.position.z = 0.0; //projection on floor plane   

        apriltag1_pose_wrt_map.pose.orientation.x = tf_apriltag1_wrt_map.transform.rotation.x;
        apriltag1_pose_wrt_map.pose.orientation.y = tf_apriltag1_wrt_map.transform.rotation.y;
        apriltag1_pose_wrt_map.pose.orientation.z = tf_apriltag1_wrt_map.transform.rotation.z;
        apriltag1_pose_wrt_map.pose.orientation.w = tf_apriltag1_wrt_map.transform.rotation.w;





        auto goal_msg = NavigateToPoseAction::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = 2.0; 
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
