//
// Created by ryuzo on 2023/02/16.
//
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <mutex>

#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "stear_node/stear_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "utilities/can_utils.hpp"

#define pi 3.14159265

/*
 * MCU Placement
 *
 * v2   v1
 *
 * v3   v4
 *
 */

namespace stear_node {

    StearNode::StearNode(const rclcpp::NodeOptions &options) : Node("er_angle_node", options) {
        using namespace std::chrono_literals;

        declare_parameter("interval_ms", 10);
        interval_ms = this->get_parameter("interval_ms").as_int();
        _publisher = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("/can_tx", _qos);
        _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                [this] { _publisher_callback(); }
        );
        _subscription = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                _qos,
                std::bind(&StearNode::_subscriber_callback, this,  std::placeholders::_1));
    }

    void StearNode::_subscriber_callback(geometry_msgs::msg::Twist msg) {
        float x_val = msg.linear.x;
        float y_val = msg.linear.x;
        float omega = msg.angular.z;

        float v1, v2, v3, v4 = 0;
        float o1, o2, o3, o4 = 0;
        float R = 0.4;
        float vk[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
        for (int i = 0; i < 4; ++i) {
            vk[i][1] = x_val - omega*R*sin((2*i + 1)*pi/4);
            vk[i][2] = y_val + omega*R*cos((2*i + 1)*pi/4);
            vk[i][0] = atan2(vk[i][2],vk[i][1]);
        }
        v1 = sqrt(vk[0][1]*vk[0][1]+vk[0][2]*vk[0][2]);
        v2 = sqrt(vk[1][1]*vk[1][1]+vk[1][2]*vk[1][2]);
        v3 = sqrt(vk[2][1]*vk[2][1]+vk[2][2]*vk[2][2]);
        v4 = sqrt(vk[3][1]*vk[3][1]+vk[3][2]*vk[3][2]);

        o1 = vk[0][0];
        o2 = vk[1][0];
        o3 = vk[2][0];
        o4 = vk[3][0];

        o1 = -o1;
        o2 = -o2;
        o3 = -o3;
        o4 = -o4;

        uint8_t _send_bytes[32];
        uint8_t _tmp_bytes[4];
        convert_float_to_byte(v1, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i] = _tmp_bytes[i];
        }
        convert_float_to_byte(v2, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 4] = _tmp_bytes[i];
        }
        convert_float_to_byte(v3, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 8] = _tmp_bytes[i];
        }
        convert_float_to_byte(v4, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 12] = _tmp_bytes[i];
        }

        uint8_t _tmp_bytes_angle[4];
        convert_float_to_byte(o1, _tmp_bytes_angle);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 16] = _tmp_bytes_angle[i];
        }
        convert_float_to_byte(o2, _tmp_bytes_angle);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 4 + 16] = _tmp_bytes_angle[i];
        }
        convert_float_to_byte(o3, _tmp_bytes_angle);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 8 + 16] = _tmp_bytes_angle[i];
        }
        convert_float_to_byte(o4, _tmp_bytes_angle);
        for (int i = 0; i < 4; ++i) {
            send_bytes[i + 12 + 16] = _tmp_bytes_angle[i];
        }
    }

    void StearNode::_publisher_callback() {
        auto msg = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg->canid = 0x101;
        msg->candlc = 32;
        for (int i = 0; i < 32; ++i) {
            msg->candata[i] = send_bytes[i];
        }
        _publisher->publish(*msg);
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(stear_node::StearNode)