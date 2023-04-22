//
// Created by ryuzo on 2023/02/16.
//
#include <rclcpp/rclcpp.hpp>
#include <mutex>

#include "rclcpp_components/register_node_macro.hpp"
#include "rr_manual_node/manual_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nlohmann/json.hpp"
#include "utilities/can_utils.hpp"
#include "tcp_socket_msg/srv/tcp_socket_i_ctrl.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using json = nlohmann::json;

namespace rr_manual_node {

    RRManualNode::RRManualNode(const rclcpp::NodeOptions &options) : Node("er_manual_node", options) {
        using namespace std::chrono_literals;
        tcp8011_flag = false;

        declare_parameter("interval_ms", 10);
        interval_ms = this->get_parameter("interval_ms").as_int();
        _publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", _qos);
        _publisher_angle = this->create_publisher<std_msgs::msg::Float32>("/er_angle", _qos);
        _publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("/can_tx", _qos);
        client_ = this->create_client<tcp_socket_msg::srv::TcpSocketICtrl>("/tcp_interface/tcp_register");
        manual_instruction["x"] = 0;
        manual_instruction["y"] = 0;
        manual_instruction["rad"] = 0;
        while (!client_->wait_for_service(1s)){
            if(!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(), "Manual Node Client interrupted while waiting for TCP service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for TCP service");
        }

//        rclcpp::WallRate loop_rate(500ms);
//        loop_rate.sleep();

        auto request_tcp_8011 = std::make_shared<tcp_socket_msg::srv::TcpSocketICtrl::Request>();
        request_tcp_8011->port = 8011;
        auto future_response_tcp_8011 = client_->async_send_request(request_tcp_8011, std::bind(&RRManualNode::_callback_response_tcp8011,
                                                                                                this, std::placeholders::_1));
        _subscription_tcp_8011 = this->create_subscription<std_msgs::msg::String>("/tcp_8011",
                                                                                           _qos,
                                                                                           std::bind(&RRManualNode::_subscriber_callback_tcp_8011, this, std::placeholders::_1)
                                                                                           );
        _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                [this] { _publisher_callback(); }
                );

        previous_angle = 0;
    }

    void RRManualNode::_subscriber_callback_tcp_8011(std_msgs::msg::String msg) {
        std::string content = msg.data;
        try
        {
            manual_instruction = json::parse(content);
        }
        catch (std::exception &e)
        {
            RCLCPP_INFO(this->get_logger(), "ERROR AT RRManualNode::_subscriber_callback_tcp_8011, IGNORING");
        }
    }

    void RRManualNode::_publisher_callback() {
        if (!this->tcp8011_flag){
            return;
        }
        float x_val = manual_instruction["x"];
        float y_val = manual_instruction["y"];
        float rad = manual_instruction["rad"];
        float angle = manual_instruction["angle"];
        if (!(previous_angle == angle)){
            auto msg_angle = std::make_shared<std_msgs::msg::Float32>();
            msg_angle->data = angle;
            _publisher_angle->publish(*msg_angle);
        }
        //RCLCPP_INFO(this->get_logger(), "x:%f, y:%f, rad:%f", x_val, y_val, rad);
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        msg->linear.x = x_val;
        msg->linear.y = y_val;
        msg->angular.z = rad;
        _publisher_cmd_vel->publish(*msg);
    }

    void RRManualNode::_callback_response_tcp8011(rclcpp::Client<tcp_socket_msg::srv::TcpSocketICtrl>::SharedFuture future) {
        if (future.get()->ack){
            this->tcp8011_flag = true;
        } else{
            RCLCPP_ERROR(this->get_logger(), "TCP service could not earn 8011 port");
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(rr_manual_node::RRManualNode)
