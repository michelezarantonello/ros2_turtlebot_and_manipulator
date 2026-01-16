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
        RCLCPP_INFO(this->get_logger(), "Manipulation node started. ##################### ");
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/pre_grasp_pose", 10, 
            std::bind(&Manipulation::move_to_pre_grasp, this, std::placeholders::_1));
        init_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&Manipulation::init_moveit, this));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriptions to cubes color detection topics 
        red_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/color_detector/red_cube", 10,
            std::bind(&Manipulation::red_callback, this, std::placeholders::_1)
        );
        blue_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/color_detector/blue_cube", 10,
            std::bind(&Manipulation::blue_callback, this, std::placeholders::_1)
        );
   }


private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr red_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr blue_subscription_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    bool returned_home = false;
    geometry_msgs::msg::PoseStamped pre_grasp_pose;
    geometry_msgs::msg::PoseStamped pre_drop_pose_wrt_base;
    geometry_msgs::msg::PoseStamped pre_grasp_pose10_wrt_base;

    void red_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)    // Callback for red cube position from camera
    {
        // Log only if the arm has returned home
        if(returned_home) {
            RCLCPP_INFO(get_logger(), "Red cube position received: [%.3f, %.3f]", 
                        msg->point.x, msg->point.y);
        }
    }
    void blue_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)   // Callback for blue cube position from camera
    {
        // Log only if the arm has returned home
        if(returned_home) {
            RCLCPP_INFO(get_logger(), "Blue cube position received: [%.3f, %.3f]", 
                        msg->point.x, msg->point.y);
        }
    }

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

            RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized ##################### ");
            init_timer_->cancel();
        } catch (const std::runtime_error &e){
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface could not be initialized: %s ##################### ", e.what());
        }
    }

    void move_to_pre_grasp(const geometry_msgs::msg::PoseStamped::SharedPtr pre_grasp_pose_msg) // Move to pre-grasp position (cube1 position)
    {    
        if (!arm_group_ || !gripper_group_ || returned_home) return;

        RCLCPP_INFO(get_logger(), "Received pre-grasp pose, planning motion ##################### ");

        // Set path constraints
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
            arm_group_->clearPoseTargets();
            RCLCPP_INFO(get_logger(), "Motion executed successfully: ir_arm moved to pre grasp pose. Opening gripper ##################### ");
            open_gripper();
        } 
        else{
            RCLCPP_ERROR(get_logger(), "Planning failed. Not available to move to pre grasp pose. ##################### ");
        }
    }

    void open_gripper()     // Open gripper before descending
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper opened succesfully ##################### ");
            cartesian_descend();
        }else{
            RCLCPP_ERROR(get_logger(), "Gripper failed opening. ##################### ");
        }
    }

    void cartesian_descend()    // Descend to grasp cube1
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian descend ##################### ");

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

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.2f ##################### ", fraction);

        if (fraction > 0.80) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian descend executed ##################### ");
            attach_cube();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian path failed ##################### ");
        }
    }

    void attach_cube()  // Attach cube1 in the simulation
    {
        RCLCPP_INFO(get_logger(), "Attaching cube1 to end-effector ##################### ");

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
        RCLCPP_INFO(get_logger(), "cube1 attached to end-effector ##################### ");
        close_gripper();
    }

    void close_gripper() // Close gripper to grasp cube1
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        std::map<std::string, double> gripper_joints;
        gripper_joints["robotiq_85_left_knuckle_joint"] = 0.10; // Setting a lower value, since "close" cannot be reached
        gripper_group_->setJointValueTarget(gripper_joints);

        RCLCPP_INFO(get_logger(), "Gripper attempt to close ##################### ");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed succesfully ##################### ");
            cartesian_ascend();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to close gripper ##################### ");
        }
    }

    void cartesian_ascend() // Ascend after grasping cube1
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian ascend ##################### ");

        std::vector<geometry_msgs::msg::Pose> waypoints;

        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose;
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.05;    
        const bool avoid_collisions = false;
        RCLCPP_INFO(get_logger(), "Planning frame: %s ##################### ", arm_group_->getPlanningFrame().c_str());
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.2f ##################### ", fraction);

        if (fraction > 0.80) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian ascend executed ##################### ");
            move_to_pre_drop();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian ascend path failed ##################### ");
        }
    }

    void move_to_pre_drop() // Move to pre-drop position 
    {
        arm_group_->clearPathConstraints();
        arm_group_->setMaxVelocityScalingFactor(0.1); // Slow movement so the cube doesn't fall
        arm_group_->setMaxAccelerationScalingFactor(0.2); // Slow movement so the cube doesn't fall

        RCLCPP_INFO(this->get_logger(), "Starting move to pre drop ##################### ");
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped pre_drop_pose_wrt_tag;
        // Pre drop position is near the other cube
        pre_drop_pose_wrt_tag.header.frame_id = "tag36h11:10";
        pre_drop_pose_wrt_tag.header.stamp = this->get_clock()->now();
        pre_drop_pose_wrt_tag.pose.position.x = -0.005; // Trial and error 
        pre_drop_pose_wrt_tag.pose.position.y = -0.10; // Trial and error 
        pre_drop_pose_wrt_tag.pose.position.z = 0.20; // Trial and error 
        pre_drop_pose_wrt_tag.pose.orientation.w = 1.0;

        try
        {
            tf_buffer_->transform(pre_drop_pose_wrt_tag, pre_drop_pose_wrt_base, "base_link", tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "pre_drop_pose_wrt_tag", "pre_drop_pose_wrt_base ##################### ", ex.what());
            return;
        }

        pre_drop_pose_wrt_base.header.stamp = this->get_clock()->now();
        pre_drop_pose_wrt_base.header.frame_id = "base_link";

        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, M_PI / 2.0);
        q.normalize();

        pre_drop_pose_wrt_base.pose.orientation = tf2::toMsg(q);
        arm_group_->setPoseTarget(pre_drop_pose_wrt_base);
        RCLCPP_INFO(this->get_logger(), "Planning move to pre drop ##################### ");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            RCLCPP_INFO(this->get_logger(), "Move to pre drop executed ##################### ");
            arm_group_->clearPoseTargets();

            // Wait for arm to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            drop_cube();
        } 
        else{
            RCLCPP_ERROR(get_logger(), "Not able to move to pre drop ##################### ");
            // Try again
            move_to_pre_drop();
        }
    }

    void drop_cube()    // Drop cube1
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), " Cube1 dropped succesfully ##################### ");
            arm_group_->detachObject("cube1");
            RCLCPP_INFO(get_logger(), " Cube1 detached ##################### ");
            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            std::vector<std::string> object_ids;
            object_ids.push_back("cube1");
            planning_scene_interface.removeCollisionObjects(object_ids);
        
            RCLCPP_INFO(get_logger(), "Cube1 removed from planning scene ##################### ");
            
            close_gripper4();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to drop cube1 ##################### ");
        }
    }

    void close_gripper4()   // Close gripper after dropping cube1 (to ensure a safe start state for the next grasp)
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("close");
        RCLCPP_INFO(get_logger(), "Gripper attempt to close ##################### ");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed successfully ##################### ");
            move_to_pre_grasp_cube10();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to close gripper ##################### ");
        }
    }

    void move_to_pre_grasp_cube10() // Move to pre-grasp position of other cube (cube10)
    {
        arm_group_->clearPathConstraints();
        arm_group_->setMaxVelocityScalingFactor(0.3); // Default speed
        arm_group_->setMaxAccelerationScalingFactor(0.3); // Default acceleration
        RCLCPP_INFO(this->get_logger(), "Starting move to pre grasp ##################### ");
        
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped pre_grasp_pose_wrt_tag10;
        pre_grasp_pose_wrt_tag10.header.frame_id = "tag36h11:10";
        pre_grasp_pose_wrt_tag10.header.stamp = rclcpp::Time(0);
        pre_grasp_pose_wrt_tag10.pose.position.x = -0.006;  // Trial and error
        pre_grasp_pose_wrt_tag10.pose.position.y = 0.03;    // Trial and error
        pre_grasp_pose_wrt_tag10.pose.position.z = 0.21;    // Trial and error
        pre_grasp_pose_wrt_tag10.pose.orientation.w = 1.0;
        

        try
        {
            tf_buffer_->transform(pre_grasp_pose_wrt_tag10, pre_grasp_pose10_wrt_base, "base_link", tf2::durationFromSec(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", "pre_grasp_pose_wrt_tag10", "pre_grasp_pose10_wrt_base ##################### ", ex.what());
            return;
        }
        
        pre_grasp_pose10_wrt_base.header.stamp = this->get_clock()->now();
        pre_grasp_pose10_wrt_base.header.frame_id = "base_link";
        
        tf2::Quaternion q;
        q.setRPY(M_PI, 0.0, M_PI / 2.0);
        q.normalize();
        

        pre_grasp_pose10_wrt_base.pose.orientation = tf2::toMsg(q);
        arm_group_->setPoseTarget(pre_grasp_pose10_wrt_base);
        RCLCPP_INFO(this->get_logger(), "Planning movement to pre grasp cube10 ##################### ");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            RCLCPP_INFO(this->get_logger(), "Successfully moved to pre grasp cube10 ##################### ");
            arm_group_->clearPoseTargets();

            open_gripper2(); 
        } 
        else{
            RCLCPP_ERROR(get_logger(), "Not able to move to pre grasp cube10 ##################### ");
            // Try again
            move_to_pre_grasp_cube10();
        }
    }

    void open_gripper2()    // Open gripper before descending to grasp cube10
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        RCLCPP_INFO(get_logger(), "Gripper attempt to open ##################### ");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper opened succesfully ##################### ");

            cartesian_descend2();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to open gripper ##################### ");
        }
    }
    

    void cartesian_descend2()   // Descend to grasp cube10
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian descend to cube10 ##################### ");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose10_wrt_base;
        target_pose.pose.position.z -= 0.07;    // Trial and error 
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.005;    
        const bool avoid_collisions = false;
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.2f ##################### ", fraction);

        if (fraction > 0.80) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian descend to cube10 executed ##################### ");

            attach_cube2();
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian descend to cube10 failed ##################### ");
        }
    }

    void attach_cube2() // Attach cube10 in the simulation
    {
        RCLCPP_INFO(get_logger(), "Attaching cube10 to end-effector ##################### ");

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
        RCLCPP_INFO(get_logger(), "cube10 attached to end-effector ##################### ");
        
        float gripper_close_value = 0.08; // Initial close value (set it less than 0.1)
        close_gripper2(gripper_close_value);
    }

    void close_gripper2(float value)    // Close gripper to grasp cube10 with retry mechanism
    {
        if (!gripper_group_) return;
        
        RCLCPP_INFO(get_logger(), "Trying closing gripper with joint value: %.3f ##################### ", value);

        gripper_group_->setStartStateToCurrentState(); 
        std::map<std::string, double> gripper_joints;
        gripper_joints["robotiq_85_left_knuckle_joint"] = value; // Adjusted close value
        gripper_group_->setJointValueTarget(gripper_joints);
        
        RCLCPP_INFO(get_logger(), "Gripper attempt to close ##################### ");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed succesfully ##################### ");
            
        }else{
            RCLCPP_ERROR(get_logger(), "Gripper failed closing, reducing value ##################### ");
        
            // Wait for gripper state to settle
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // Retry closing with a slightly decreased value
            if(value > 0.03) {
                value -= 0.002; // Decrease by 0.002
                close_gripper2(value);
                return;
            } else {
                RCLCPP_ERROR(get_logger(), "Aborting gripper close after multiple attempts ##################### ");
            }

        }
        cartesian_ascend2();
    }

    void cartesian_ascend2()    // Ascend after grasping cube10
    {
        if (!arm_group_) return;

        RCLCPP_INFO(get_logger(), "Starting Cartesian ascend ##################### ");

        std::vector<geometry_msgs::msg::Pose> waypoints;

        arm_group_->setStartStateToCurrentState();
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose = pre_grasp_pose10_wrt_base;
        target_pose.pose.position.z += 0.06;    // Trial and error
        waypoints.push_back(target_pose.pose);

        moveit_msgs::msg::RobotTrajectory trajectory;

        const double step = 0.005;    
        const bool avoid_collisions = false;
        RCLCPP_INFO(get_logger(), "Planning frame: %s ##################### ", arm_group_->getPlanningFrame().c_str());
        double fraction = arm_group_->computeCartesianPath(
            waypoints,
            step,
            trajectory,
            avoid_collisions);

        RCLCPP_INFO(get_logger(), "Cartesian path fraction: %.2f ##################### ", fraction);

        if (fraction > 0.3) {
            arm_group_->execute(trajectory);
            RCLCPP_INFO(get_logger(), "Cartesian ascend executed ##################### ");

            // Wait for arm to settle after partial trajectory
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            float orientation_tolerance = M_PI/180*30; // Imposing initial tolerance 
            move_to_pre_drop2(orientation_tolerance);
        } else {
            RCLCPP_ERROR(get_logger(), "Cartesian ascend path failed ##################### ");
            // Try again
            cartesian_ascend2();
        }
    }

    void move_to_pre_drop2(float orientation_tolerance) // Move to pre-drop position of cube10 with retry mechanism
    {
        arm_group_->clearPathConstraints();

        // Wait a bit more to ensure state is updated
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        arm_group_->setStartStateToCurrentState();

        arm_group_->setMaxVelocityScalingFactor(0.1); // Slow movement so the cube doesn't fall
        arm_group_->setMaxAccelerationScalingFactor(0.2); // Slow movement so the cube doesn't fall

        RCLCPP_INFO(this->get_logger(), "Starting move to pre drop 2 ##################### ");
        // Set path constraints
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
        ocm.absolute_x_axis_tolerance = orientation_tolerance;
        ocm.absolute_y_axis_tolerance = orientation_tolerance;
        ocm.absolute_z_axis_tolerance = 3.14;
        ocm.weight = 1.0; 

        moveit_msgs::msg::Constraints constraints;
        constraints.joint_constraints.push_back(jc_shoulder_pan);
        constraints.joint_constraints.push_back(jc_shoulder_lift);
        constraints.orientation_constraints.push_back(ocm);
        arm_group_->setPathConstraints(constraints);

        RCLCPP_INFO(get_logger(), "Constraints setted ##################### ");

        arm_group_->setPoseTarget(pre_grasp_pose);
        RCLCPP_INFO(this->get_logger(), "Planning movement to pre-drop 2 ##################### ");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = arm_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success) {
            arm_group_->execute(my_plan); 
            RCLCPP_INFO(this->get_logger(), "Moved to pre-drop position 2 ##################### ");
            arm_group_->clearPoseTargets();
            
            drop_cube2();
        } 
        else{
            RCLCPP_ERROR(get_logger(), "Not able to move to pre-drop position 2 ##################### ");
            // Retry with increased orientation tolerance
            if(orientation_tolerance < M_PI) {
                orientation_tolerance += M_PI/180*10; // Increase by 10 degrees
                RCLCPP_INFO(get_logger(), "Retrying move to pre drop 2 with orientation tolerance: %.2f degrees ##################### ", orientation_tolerance * (180.0/M_PI));
                move_to_pre_drop2(orientation_tolerance);
            }
            else {
                RCLCPP_ERROR(get_logger(), "Aborting move to pre drop after multiple attempts #####################");
                drop_cube2();
            }
        }
    }

    void drop_cube2()   // Drop cube10
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("open");
        RCLCPP_INFO(get_logger(), "Gripper attempt to open ##################### ");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "cube10 dropped succesfully ##################### ");
            arm_group_->detachObject("cube10");
            RCLCPP_INFO(get_logger(), "cube10 detached ##################### ");

            moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
            std::vector<std::string> object_ids;
            object_ids.push_back("cube10");
            planning_scene_interface.removeCollisionObjects(object_ids);
            RCLCPP_INFO(get_logger(), "cube10 removed from planning scene ##################### ");

            close_gripper3();
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to drop cube10 ##################### ");
        }
    }

    void close_gripper3()   // Close gripper after dropping cube10
    {
        if (!gripper_group_) return;

        gripper_group_->setStartStateToCurrentState();
        gripper_group_->setNamedTarget("close");
        RCLCPP_INFO(get_logger(), "Gripper attempt to close ##################### ");
        bool success = gripper_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Gripper closed successfully ##################### ");
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to close gripper ##################### ");
        }
        return_home();
    }

    void return_home()  // Return to home position
    {
        arm_group_->setMaxVelocityScalingFactor(0.5); // Increased speed
        arm_group_->setMaxAccelerationScalingFactor(0.5); // Increased acceleration
        arm_group_->clearPathConstraints();
        arm_group_->setStartStateToCurrentState();
        arm_group_->setNamedTarget("home");
        RCLCPP_INFO(this->get_logger(), "Planning returning to home position ##################### ");
        bool success = arm_group_->move() == moveit::core::MoveItErrorCode::SUCCESS;
        if (success){
            RCLCPP_INFO(get_logger(), "Returned to home succesfully ##################### ");
            // Enable acquisition of cube positions
            returned_home = true;                
        }else{
            RCLCPP_ERROR(get_logger(), "Failed to return to home ##################### ");
            // Try again
            return_home();
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