#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class ManageLifecycleNodesClient : public rclcpp::Node
{
public:
    ManageLifecycleNodesClient() : Node("manage_lifecycle_nodes_client")
    {
        localization_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("lifecycle_manager_localization");
        navigation_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("lifecycle_manager_navigation");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;
        

        localization_client_->wait_for_service();
        
        auto result_localization = localization_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_localization) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "localization startup completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "localization startup failed");
        }

        navigation_client_->wait_for_service();

        auto result_navigation = navigation_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_navigation) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "navigation startup completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "navigation startup failed");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr localization_client_;
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client_;
};