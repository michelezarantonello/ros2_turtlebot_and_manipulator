
#include <cmath>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Move_to_pre_grasp_pose : public rclcpp::Node, public std::enable_shared_from_this<Move_to_pre_grasp_pose>
{
public:
   Move_to_pre_grasp_pose()  : Node("move_to_pre_grasp_pose")
   {
    RCLCPP_INFO(this->get_logger(), "Move_to_pre_grasp_pose node started.");
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pre_grasp_pose", 10, 
        std::bind(&Move_to_pre_grasp_pose::topic_callback, this, std::placeholders::_1));
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
    move_group->setMaxVelocityScalingFactor(0.3); 
    move_group->setMaxAccelerationScalingFactor(0.3); 
    move_group->setPlanningTime(8.0);
   }

private:
   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
   bool motion_executed_{false};

   void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pre_grasp_pose_msg)
   {    
    if (motion_executed_){
        return;
    }

    RCLCPP_INFO(get_logger(), "Received pre-grasp pose, planning motion");
    
    move_group->setPoseTarget(*pre_grasp_pose_msg);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    if (success) {
        move_group->execute(my_plan); 
        motion_executed_ = true;
        move_group->clearPoseTargets();
        RCLCPP_INFO(get_logger(), "Motion executed successfully");
    } 
    else{
            RCLCPP_ERROR(get_logger(), "Planning failed");
    }
    }
};

int main(int argc, char * argv[])
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<Move_to_pre_grasp_pose>());
   rclcpp::shutdown();
   return 0;
}
