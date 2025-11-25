#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
    }

    void send_goal()
    {
        if(!action_client_->wait_for_action_server(std::chrono::seconds(7))){
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }
        geometry_msgs::msg::TransformStamped tf_apriltag0_wrt_map;
        geometry_msgs::msg::TransformStamped tf_apriltag1_wrt_map;
        geometry_msgs::msg::PoseStamped goal_pose_wrt_map;
        geometry_msgs::msg::PoseStamped apriltag0_pose_wrt_map;
        geometry_msgs::msg::PoseStamped apriltag1_pose_wrt_map;

        try{tf_apriltag0_wrt_map = tf_buffer_->lookupTransform("map", "tag_0", tf2::TimePointZero);} 
        catch(const tf2::TransformException & ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "tag_0", "map", ex.what());
        return;
        }
        apriltag0_pose_wrt_map.header.stamp = this->get_clock()->now();
        apriltag0_pose_wrt_map.header.frame_id = "map";

        apriltag0_pose_wrt_map.pose.position.x = tf_apriltag0_wrt_map.transform.translation.x;
        apriltag0_pose_wrt_map.pose.position.y = tf_apriltag0_wrt_map.transform.translation.y;
        apriltag0_pose_wrt_map.pose.position.z = tf_apriltag0_wrt_map.transform.translation.z; 

        apriltag0_pose_wrt_map.pose.orientation.x = tf_apriltag0_wrt_map.transform.rotation.x;
        apriltag0_pose_wrt_map.pose.orientation.y = tf_apriltag0_wrt_map.transform.rotation.y;
        apriltag0_pose_wrt_map.pose.orientation.z = tf_apriltag0_wrt_map.transform.rotation.z;
        apriltag0_pose_wrt_map.pose.orientation.w = tf_apriltag0_wrt_map.transform.rotation.w;

        
        try{tf_apriltag1_wrt_map = tf_buffer_->lookupTransform("map", "tag_1", tf2::TimePointZero);} 
        catch(const tf2::TransformException & ex){
        RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "tag_1", "map", ex.what());
        return;
        }
        apriltag1_pose_wrt_map.header.stamp = this->get_clock()->now();
        apriltag1_pose_wrt_map.header.frame_id = "map";

        apriltag1_pose_wrt_map.pose.position.x = tf_apriltag1_wrt_map.transform.translation.x;
        apriltag1_pose_wrt_map.pose.position.y = tf_apriltag1_wrt_map.transform.translation.y;
        apriltag1_pose_wrt_map.pose.position.z = tf_apriltag1_wrt_map.transform.translation.z; 

        apriltag1_pose_wrt_map.pose.orientation.x = tf_apriltag1_wrt_map.transform.rotation.x;
        apriltag1_pose_wrt_map.pose.orientation.y = tf_apriltag1_wrt_map.transform.rotation.y;
        apriltag1_pose_wrt_map.pose.orientation.z = tf_apriltag1_wrt_map.transform.rotation.z;
        apriltag1_pose_wrt_map.pose.orientation.w = tf_apriltag1_wrt_map.transform.rotation.w;

        goal_pose_wrt_map.header.stamp = this->get_clock()->now();
        goal_pose_wrt_map.header.frame_id = "map";

        goal_pose_wrt_map.pose.position.x = (apriltag1_pose_wrt_map.pose.position.x + apriltag0_pose_wrt_map.pose.position.x)/2;
        goal_pose_wrt_map.pose.position.y = (apriltag1_pose_wrt_map.pose.position.y + apriltag0_pose_wrt_map.pose.position.y)/2;
        goal_pose_wrt_map.pose.position.z = (apriltag1_pose_wrt_map.pose.position.z + apriltag0_pose_wrt_map.pose.position.z)/2;
        //random orientation---to fix
        goal_pose_wrt_map.pose.orientation.x = apriltag1_pose_wrt_map.pose.orientation.x;
        goal_pose_wrt_map.pose.orientation.y = apriltag1_pose_wrt_map.pose.orientation.y;
        goal_pose_wrt_map.pose.orientation.z = apriltag1_pose_wrt_map.pose.orientation.z;
        goal_pose_wrt_map.pose.orientation.w = apriltag1_pose_wrt_map.pose.orientation.w;        

        auto goal_msg = NavigateToPoseAction::Goal();

        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        goal_msg.pose.pose.position = goal_pose_wrt_map.pose.position; 
        
        goal_msg.pose.pose.orientation = goal_pose_wrt_map.pose.orientation;
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();

        send_goal_options.feedback_callback = std::bind(&Nav2ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.goal_response_callback = std::bind(&Nav2ActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&Nav2ActionClient::result_callback, this, std::placeholders::_1);
        
        RCLCPP_INFO(this->get_logger(), "Sending goal to action server...");
        action_client_->async_send_goal(goal_msg, send_goal_options);
        
    }

private:
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by action server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by action server");
        }
    }

    void feedback_callback(GoalHandle::SharedPtr,  const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),  "Navigation feedback received: distance_remaining = %.2f", feedback->distance_remaining);
    }

    void result_callback(const GoalHandle::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    }
};
