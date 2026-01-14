#include <rclcpp/rclcpp.hpp>                            
#include <geometry_msgs/msg/pose_stamped.hpp>             
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp> 
#include <moveit/planning_scene_interface/planning_scene_interface.hpp> 
#include <moveit_msgs/msg/robot_trajectory.hpp>    
#include <moveit_msgs/msg/planning_scene.hpp>      
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <memory>                                       
#include <chrono>                                       
#include <functional>  


#define ARM_GROUP "ir_arm"
#define GRIPPER_GROUP "ir_gripper"

using namespace std::chrono_literals;

class Manipulation : public rclcpp::Node
{
public:
   explicit Manipulation(const rclcpp::NodeOptions & options)  : Node("manipulation", options)
   {
        RCLCPP_INFO(this->get_logger(), "Manipulation node started.");
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pre_grasp_pose", 10, 
            std::bind(&Manipulation::move_to_pre_grasp, this, std::placeholders::_1));
        init_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&Manipulation::init_moveit, this));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
   }


private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    bool arm_moved_ = false;
    geometry_msgs::msg::PoseStamped pre_grasp_pose;
    geometry_msgs::msg::PoseStamped pre_drop_pose_wrt_base;
    geometry_msgs::msg::PoseStamped pre_grasp_pose10_wrt_base;


    void init_moveit()
    {
        if (arm_group_ && gripper_group_) return;

        try{
            auto node = this->shared_from_this();
            arm_group_ =std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, ARM_GROUP);
            arm_group_->setPoseReferenceFrame("base_link");
            arm_group_->setEndEffectorLink("tool0");
            arm_group_->setMaxVelocityScalingFactor(0.3);
            arm_group_->setMaxAccelerationScalingFactor(0.3);
            arm_group_->setPlanningTime(15.0);
            
            gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, GRIPPER_GROUP);

            RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized");
            init_timer_->cancel();
        } catch (const std::runtime_error &e){
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface could not be initialized: %s", e.what());
        }
    }

    void move_to_pre_grasp(const geometry_msgs::msg::PoseStamped::SharedPtr pre_grasp_pose_msg)
    {    
        if (!arm_group_ || !gripper_group_ || arm_moved_) return;

        RCLCPP_INFO(get_logger(), "Received pre-grasp pose, planning motion");
        moveit_msgs::msg::JointConstraint jc_shoulder_pan;
        moveit_msgs::msg::JointConstraint jc_shoulder_lift;
        jc_shoulder_pan.joint_name = "shoulder_pan_joint";
        jc_shoulder_pan.position = -M_PI/2;
        jc_shoulder_pan.tolerance_above = M_PI/180*100;
        jc_shoulder_pan.tolerance_below = M_PI/180*100;
        jc_shoulder_pan.weight = 1.0;

        jc_shoulder_lift.joint_name = "shoulder_lift_joint";
        jc_shoulder_lift.position = -M_PI/2;
        jc_shoulder_lift.tolerance_above = M_PI/180*40;
        jc_shoulder_lift.tolerance_below = M_PI/180*25;
        jc_shoulder_lift.weight = 1.0;

        moveit_msgs::msg::Constraints constraints;
        constraints.joint_constraints.push_back(jc_shoulder_pan);
        constraints.joint_constraints.push_back(jc_shoulder_lift);
        arm_group_->setPathConstraints(constraints);
        RCLCPP_INFO(get_logger(), "Constraints setted.");
        arm_group_->setStartStateToCurrentState();
        pre_grasp_pose = *pre_grasp_pose_msg;
        arm_group_->setPoseTarget(pre_grasp_pose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            arm_moved_ = true;
            arm_group_->clearPoseTargets();
            RCLCPP_INFO(get_logger(), "Motion executed successfully: ir_arm moved to pre grasp pose. Opening gripper...");
            open_gripper();
        } 
        else{
                RCLCPP_ERROR(get_logger(), "Planning failed. Not available to move to pre grasp pose.");
        }
    }

    void open_gripper()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper opened succesfully");
            cartesian_descend();
        }else{
            RCLCPP_ERROR(get_logger(), "Gripper failed opening.");
        }

    }

    void cartesian_descend()
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian descend");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose;
        target_pose.pose.position.z -= 0.08; // 8cm descend
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.05;    
        const bool avoid_collisions = false;
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "############Cartesian path fraction: %.2f", fraction);

        if (fraction > 0.80) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian descend executed#################################");
            attach_cube();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian path failed##################################");
        }
    }

    void attach_cube()
    {
        RCLCPP_INFO(get_logger(), "Attaching cube1 to end-effector");

        arm_group_->attachObject(
            "cube1",
            arm_group_->getEndEffectorLink(),
            {"robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_inner_knuckle_link"
            });
        RCLCPP_INFO(get_logger(), "cube1 attached to end-effector");
        close_gripper();
    }

    void close_gripper()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        std::map<std::string, double> gripper_joints;
        gripper_joints["robotiq_85_left_knuckle_joint"] = 0.10;// gripper_group_->setNamedTarget("close") do not reach the close goal
        gripper_group_->setJointValueTarget(gripper_joints);
        // gripper_group_->move(); 
        // RCLCPP_INFO(get_logger(), "Gripper closing maybe###########################");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed succesfully######################");
            cartesian_ascend();
        }else{
            RCLCPP_ERROR(get_logger(), "Gripper failed closing.########################");
        }
    }



    void cartesian_ascend()
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian ascend");

        std::vector<geometry_msgs::msg::Pose> waypoints;

        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose;
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.05;    
        const bool avoid_collisions = false;
        RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_group_->getPlanningFrame().c_str());
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.2f", fraction);

        if (fraction > 0.80) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian ascend executed");
            move_to_pre_drop();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian ascend path failed");
        }
    }

    void move_to_pre_drop()
    {
        arm_group_->clearPathConstraints();
        arm_group_->setMaxVelocityScalingFactor(0.1); // Slow movement for a safe drop
        arm_group_->setMaxAccelerationScalingFactor(0.2); // Slow movement for a safe drop
        RCLCPP_INFO(this->get_logger(), "STARTING MOVE TO PRE DROP ##############");
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped pre_drop_pose_wrt_tag;
        pre_drop_pose_wrt_tag.header.frame_id = "tag36h11:10";
        pre_drop_pose_wrt_tag.header.stamp = this->get_clock()->now();
        pre_drop_pose_wrt_tag.pose.position.x = -0.005; // Changed by big
        pre_drop_pose_wrt_tag.pose.position.y = -0.10; // Changed by big
        pre_drop_pose_wrt_tag.pose.position.z = 0.20; // Changed by big

        pre_drop_pose_wrt_tag.pose.orientation.w = 1.0;

        try
        {
            tf_buffer_->transform(pre_drop_pose_wrt_tag, pre_drop_pose_wrt_base, "base_link", tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "pre_drop_pose_wrt_tag", "pre_drop_pose_wrt_base", ex.what());
            return;
        }

        pre_drop_pose_wrt_base.header.stamp = this->get_clock()->now();
        pre_drop_pose_wrt_base.header.frame_id = "base_link";

        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, M_PI / 2.0);
        q.normalize();

        pre_drop_pose_wrt_base.pose.orientation = tf2::toMsg(q);
        arm_group_->setPoseTarget(pre_drop_pose_wrt_base);
        RCLCPP_INFO(this->get_logger(), "MOVING TO PRE DROP ##############");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            RCLCPP_INFO(this->get_logger(), "MOVING TO PRE DROP ##############");
            arm_group_->clearPoseTargets();
            
            // Wait for state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            drop_cube();
        } 
        else{
                RCLCPP_ERROR(get_logger(), "NOT ABLE TO MOVE TO PRE DROP ##############");
        }
    }

    void drop_cube()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), " Cube dropped succesfully########################");
            arm_group_->detachObject("cube1");
            RCLCPP_INFO(get_logger(), " Cube1 DETACHED ########################");
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            std::vector<std::string> object_ids;
            object_ids.push_back("cube1");
            planning_scene_interface.removeCollisionObjects(object_ids);
        
            RCLCPP_INFO(get_logger(), "Cube1 REMOVED from planning scene");
            // Wait for gripper state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            close_gripper4();
        }else{
            RCLCPP_ERROR(get_logger(), "Cube failed drop##########################.");
        }
    }

    void close_gripper4()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("close");
        RCLCPP_INFO(get_logger(), "Gripper attempt to close ###########################");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed successfully.");
            move_to_pre_grasp_cube10();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to close gripper.");
        }
    }

    void move_to_pre_grasp_cube10()
    {
        arm_group_->clearPathConstraints();
        arm_group_->setMaxVelocityScalingFactor(0.3); // Default speed
        arm_group_->setMaxAccelerationScalingFactor(0.3); // Default acceleration
        RCLCPP_INFO(this->get_logger(), "STARTING MOVE TO PRE GRASP ##############");
        
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped pre_grasp_pose_wrt_tag10;
        pre_grasp_pose_wrt_tag10.header.frame_id = "tag36h11:10";
        pre_grasp_pose_wrt_tag10.header.stamp = rclcpp::Time(0);
        pre_grasp_pose_wrt_tag10.pose.position.x = -0.006;//br trial and error // Changed by big, prev=-0.008
        pre_grasp_pose_wrt_tag10.pose.position.y = 0.03; 
        pre_grasp_pose_wrt_tag10.pose.position.z = 0.21;//0.20 

        pre_grasp_pose_wrt_tag10.pose.orientation.w = 1.0;
        

        try
        {
            tf_buffer_->transform(pre_grasp_pose_wrt_tag10, pre_grasp_pose10_wrt_base, "base_link", tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "pre_grasp_pose_wrt_tag10", "pre_grasp_pose10_wrt_base", ex.what());
            return;
        }
        
        pre_grasp_pose10_wrt_base.header.stamp = this->get_clock()->now();
        pre_grasp_pose10_wrt_base.header.frame_id = "base_link";
        
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, M_PI / 2.0);
        q.normalize();
        

        pre_grasp_pose10_wrt_base.pose.orientation = tf2::toMsg(q);
        arm_group_->setPoseTarget(pre_grasp_pose10_wrt_base);
        RCLCPP_INFO(this->get_logger(), "MOVING TO PRE GRASP10 ##############");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            RCLCPP_INFO(this->get_logger(), "SUCCESFULLY MOVED TO PRE GRASP10 ##############");
            arm_group_->clearPoseTargets();

            // Wait for arm state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            open_gripper2(); 
        } 
        else{
            RCLCPP_ERROR(get_logger(), "NOT ABLE TO MOVE TO PRE GRASP10 ##############");
            move_to_pre_grasp_cube10();
        }
    }

    void open_gripper2()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        RCLCPP_INFO(get_logger(), "Gripper attempt to open ###########################");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper opened succesfully");

            // Wait for arm state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            cartesian_descend2();
        }else{
            RCLCPP_ERROR(get_logger(), "Gripper failed opening.");
        }

    }
    

    void cartesian_descend2()
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian descend to blue cube ###################");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose10_wrt_base;
        target_pose.pose.position.z -= 0.065; // -=0.06  // Changed by big
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.005;    
        const bool avoid_collisions = false;
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "############Cartesian path fraction: %.2f", fraction);

        if (fraction > 0.80) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian descend to blue cube executed#################################");
            // Wait for state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            attach_cube2();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian descend to blue cube failed##################################");
        }
    }

    void attach_cube2()
    {
        RCLCPP_INFO(get_logger(), "Attaching cube10 to end-effector");

        arm_group_->attachObject(
            "cube10",
            arm_group_->getEndEffectorLink(),
            {"robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_inner_knuckle_link"
            });
        RCLCPP_INFO(get_logger(), "cube10 attached to end-effector");
        
        // Wait for state to settle
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        close_gripper2();
    }

    void close_gripper2()
    {
        if (!gripper_group_) return;
        
        // Wait for gripper state to fully settle before reading current state
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        gripper_group_->setStartStateToCurrentState(); //enforce bounds to get rid of this error: Joint 'robotiq_85_right_knuckle_joint' from the starting state is outside bounds by: [2.50818e-05 ] should be in the range [-0.8 ], [0 ].
        std::map<std::string, double> gripper_joints;
        gripper_joints["robotiq_85_left_knuckle_joint"] = 0.08;//0.05 gripper_group_->setNamedTarget("close") do not reach the close goal // Chaanged by big, prev = 0.08
        gripper_group_->setJointValueTarget(gripper_joints);
        
        RCLCPP_INFO(get_logger(), "Gripper attempt to close ###########################");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed succesfully######################");
            
        }else{
            RCLCPP_ERROR(get_logger(), "Gripper failed closing.########################");
            // Detach cube if gripper failed to close
            arm_group_->detachObject("cube10");
            RCLCPP_INFO(get_logger(), " Cube10 DETACHED ########################");
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            std::vector<std::string> object_ids;
            object_ids.push_back("cube10");
            planning_scene_interface.removeCollisionObjects(object_ids);
        
            RCLCPP_INFO(get_logger(), "Cube10 REMOVED from planning scene");
            // Wait for gripper state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
 
        }
        cartesian_ascend2();
    }

    void cartesian_ascend2()
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian ascend#################");

        std::vector<geometry_msgs::msg::Pose> waypoints;

        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose10_wrt_base;
        target_pose.pose.position.z += 0.065;    // Changed by big
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.005;    
        const bool avoid_collisions = false;
        RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_group_->getPlanningFrame().c_str());
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.2f", fraction);

        if (fraction > 0.3) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian ascend executed#########################");

            // Wait for arm to settle after partial trajectory
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            move_to_pre_drop2();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian ascend path failed#########################");
        }
    }

    void move_to_pre_drop2()
    {
        arm_group_->clearPathConstraints();

        // Wait a bit more to ensure state is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        arm_group_->setStartStateToCurrentState();

        arm_group_->setMaxVelocityScalingFactor(0.1); // Slow movement for a safe drop
        arm_group_->setMaxAccelerationScalingFactor(0.2); // Slow movement for a safe drop
        
        RCLCPP_INFO(this->get_logger(), "STARTING MOVE TO PRE DROP2 ##############");
        
        moveit_msgs::msg::JointConstraint jc_shoulder_pan;
        moveit_msgs::msg::JointConstraint jc_shoulder_lift;
        jc_shoulder_pan.joint_name = "shoulder_pan_joint";
        jc_shoulder_pan.position = -M_PI/2;
        jc_shoulder_pan.tolerance_above = M_PI/180*110;
        jc_shoulder_pan.tolerance_below = M_PI/180*110;
        jc_shoulder_pan.weight = 1.0;

        jc_shoulder_lift.joint_name = "shoulder_lift_joint";
        jc_shoulder_lift.position = -M_PI/2;
        jc_shoulder_lift.tolerance_above = M_PI/180*50;
        jc_shoulder_lift.tolerance_below = M_PI/180*50;
        jc_shoulder_lift.weight = 1.0;

        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = "tool0";
        ocm.header.frame_id = "base_link";
        ocm.orientation.x = 0.707;
        ocm.orientation.y = 0.0;
        ocm.orientation.z = 0.0;
        ocm.orientation.w = 0.707;
        ocm.absolute_x_axis_tolerance = M_PI/2;
        ocm.absolute_y_axis_tolerance = M_PI/2;
        ocm.absolute_z_axis_tolerance = 3.14;
        ocm.weight = 1.0; 

        moveit_msgs::msg::Constraints constraints;
        constraints.joint_constraints.push_back(jc_shoulder_pan);
        constraints.joint_constraints.push_back(jc_shoulder_lift);
        constraints.orientation_constraints.push_back(ocm);
        arm_group_->setPathConstraints(constraints);

        RCLCPP_INFO(get_logger(), "Constraints setted.");

        arm_group_->setPoseTarget(pre_grasp_pose);
        RCLCPP_INFO(this->get_logger(), "MOVING TO PRE DROP ##############");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            RCLCPP_INFO(this->get_logger(), "MOVED TO PRE DROP ##############");
            arm_group_->clearPoseTargets();
            
            drop_cube2();
        } 
        else{
            RCLCPP_ERROR(get_logger(), "NOT ABLE TO MOVE TO PRE DROP ##############");
            drop_cube2();
        }
    }

    void drop_cube2()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), " Cube 10 dropped succesfully########################");
            arm_group_->detachObject("cube10");
            RCLCPP_INFO(get_logger(), " Cube10 DETACHED ########################");
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            std::vector<std::string> object_ids;
            object_ids.push_back("cube10");
            planning_scene_interface.removeCollisionObjects(object_ids);
        
            RCLCPP_INFO(get_logger(), "Cube10 REMOVED from planning scene");
            // Wait for gripper state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            close_gripper3();
        }else{
            RCLCPP_ERROR(get_logger(), "Cube failed drop##########################.");
        }
    }

    void close_gripper3()
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("close");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed successfully.");
            return_home();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to close gripper.");
        }
    }

    void return_home()
    {
        arm_group_->setMaxVelocityScalingFactor(0.5); // Increased speed
        arm_group_->setMaxAccelerationScalingFactor(0.5); // Increased acceleration
        arm_group_->clearPathConstraints();
        arm_group_->setStartStateToCurrentState();
        arm_group_->setNamedTarget("home");
        bool success = arm_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "returned to home succesfully ###############################");
                
        }else{
            RCLCPP_ERROR(get_logger(), " failed returned to home###############################.");
        }
    }

    



};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto manipulation_node =
      std::make_shared<Manipulation>(node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(manipulation_node);

  executor.spin();   

  rclcpp::shutdown();
  return 0;
}