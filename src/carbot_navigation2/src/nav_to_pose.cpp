#include<memory>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <string>
using NavigationAction = nav2_msgs::action::NavigateToPose;


class nav_to_pose :public rclcpp::Node
{
public:
    using NavigationActionHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;

    nav_to_pose(std::string name):Node(name)
    {
        action_client = rclcpp_action::create_client<NavigationAction>(
            this, "navigate_to_pose");
        send_goal();

    }

private:
    rclcpp_action::Client<NavigationAction>::SharedPtr action_client;
    NavigationAction::Goal goal_msg;
    rclcpp_action::Client<NavigationAction>::SendGoalOptions send_goal_options;

    void send_goal()
    {
        while (!action_client->wait_for_action_server(std::chrono::seconds(5))) 
        {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "服务被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务");
        }
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 2.0f;
        goal_msg.pose.pose.position.y = 2.0f;

        using std::placeholders::_1;
        using std::placeholders::_2;

        send_goal_options.goal_response_callback =
            std::bind(&nav_to_pose::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&nav_to_pose::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&nav_to_pose::result_callback, this, _1);
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        action_client->async_send_goal(goal_msg, send_goal_options);
    }
    void goal_response_callback(
        NavigationActionHandle::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标被接受");
        }
    }    
    void feedback_callback(
        NavigationActionHandle::SharedPtr goal_handle,
        const std::shared_ptr<const NavigationAction::Feedback> feedback)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "反馈: %f", feedback->distance_remaining);
    }
    void result_callback(
        const NavigationActionHandle::WrappedResult & result)
    {
        switch (result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "成功");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果");
                break;
        }
    }
    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav_to_pose>("nav_to_pose");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


