#include <memory>                                       
#include <chrono>                                       
#include <functional>  
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp> 
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>    
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



using namespace std::chrono_literals;

class Collision_objects : public rclcpp::Node
{
public:
    Collision_objects() : Node("collision_objects")
    {
        RCLCPP_INFO(this->get_logger(), "Collision_objects node started");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&Collision_objects::addCollisions, this));
    }

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    void addCollisions()
    {
        moveit_msgs::msg::CollisionObject cube1;

        cube1.header.frame_id = "base_link";
        cube1.id = "cube1";


        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.06, 0.06, 0.10};

        geometry_msgs::msg::Pose pose_cube1;
        geometry_msgs::msg::TransformStamped tf_cube1_pose_wrt_base_link;
        if (!tf_buffer_->canTransform("base_link", "tag36h11:1", tf2::TimePointZero)) {
            RCLCPP_WARN(this->get_logger(), "TF base_link -> tag36h11:1 non available yet, waiting...");
            return;
        }
        try {
            tf_cube1_pose_wrt_base_link = tf_buffer_->lookupTransform("base_link", "tag36h11:1", tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) 
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", "tag36h11:1", "base_link", ex.what());
            return;
        }
        pose_cube1.orientation.x = tf_cube1_pose_wrt_base_link.transform.rotation.x;
        pose_cube1.orientation.y = tf_cube1_pose_wrt_base_link.transform.rotation.y;
        pose_cube1.orientation.z = tf_cube1_pose_wrt_base_link.transform.rotation.z;
        pose_cube1.orientation.w = tf_cube1_pose_wrt_base_link.transform.rotation.w;
        pose_cube1.position.x = tf_cube1_pose_wrt_base_link.transform.translation.x;
        pose_cube1.position.y = tf_cube1_pose_wrt_base_link.transform.translation.y;
        pose_cube1.position.z = tf_cube1_pose_wrt_base_link.transform.translation.z;
        pose_cube1.position.x += 0.03; //by trial and error
        pose_cube1.position.z -= 0.05;

        cube1.primitives.push_back(primitive);
        cube1.primitive_poses.push_back(pose_cube1);
        cube1.operation = cube1.ADD;

        planning_scene_interface_.applyCollisionObject(cube1);

        RCLCPP_INFO(this->get_logger(), "cube1 added to planning scene");

//same for cube10
        moveit_msgs::msg::CollisionObject cube10;
        cube10.header.frame_id = "base_link";
        cube10.id = "cube10";
        geometry_msgs::msg::Pose pose_cube10;
        geometry_msgs::msg::TransformStamped tf_cube10_pose_wrt_base_link;
        if (!tf_buffer_->canTransform("base_link", "tag36h11:10", tf2::TimePointZero)) {
            RCLCPP_WARN(this->get_logger(), "TF base_link -> tag36h11:10 non available yet, waiting...");
            return;
        }
        try {
            tf_cube10_pose_wrt_base_link = tf_buffer_->lookupTransform("base_link", "tag36h11:10", tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) 
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", "tag36h11:10", "base_link", ex.what());
            return;
        }
        pose_cube10.orientation.x = tf_cube10_pose_wrt_base_link.transform.rotation.x;
        pose_cube10.orientation.y = tf_cube10_pose_wrt_base_link.transform.rotation.y;
        pose_cube10.orientation.z = tf_cube10_pose_wrt_base_link.transform.rotation.z;
        pose_cube10.orientation.w = tf_cube10_pose_wrt_base_link.transform.rotation.w;
        pose_cube10.position.x = tf_cube10_pose_wrt_base_link.transform.translation.x;
        pose_cube10.position.y = tf_cube10_pose_wrt_base_link.transform.translation.y;
        pose_cube10.position.z = tf_cube10_pose_wrt_base_link.transform.translation.z;
        pose_cube10.position.x += 0.03; //trial and error
        pose_cube10.position.y += 0.01;
        pose_cube10.position.z -= 0.05;

        cube10.primitives.push_back(primitive);
        cube10.primitive_poses.push_back(pose_cube10);
        cube10.operation = cube10.ADD;

        planning_scene_interface_.applyCollisionObject(cube10);

        RCLCPP_INFO(this->get_logger(), "cube10 added to planning scene");

        // //ALLOWING COLLISION WITH CUBES
        // moveit_msgs::msg::PlanningScene planning_scene;
        // planning_scene.is_diff = true;
        // planning_scene.allowed_collision_matrix.entry_names.push_back("cube1");
        // moveit_msgs::msg::AllowedCollisionEntry entry;
        // entry.enabled.resize(2, true);
        // planning_scene.allowed_collision_matrix.entry_values.push_back(entry);
        // planning_scene.allowed_collision_matrix.entry_names.push_back("robotiq_85_left_finger_link");
        // planning_scene.allowed_collision_matrix.entry_names.push_back("robotiq_85_right_finger_link");

        // planning_scene_interface_.applyPlanningScene(planning_scene);

        timer_->cancel();  
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Collision_objects>());
    rclcpp::shutdown();
    return 0;
}