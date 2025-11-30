#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class ManageLifecycleNodesClient : public rclcpp::Node
{
public:
    ManageLifecycleNodesClient() : Node("manage_navigation_nodes")
    {
    
        navigation_client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("/lifecycle_manager_navigation/manage_nodes");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        request->command = nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP;

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
  
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr navigation_client_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ManageLifecycleNodesClient>();
    node->send_request();

    rclcpp::shutdown();
    return 0;
}