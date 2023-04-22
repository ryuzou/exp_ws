//
// Created by ryuzo on 2023/02/16.
//
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <mutex>

#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mechanam_node/mechanam_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "utilities/can_utils.hpp"

namespace mechanam_node {

    MechanamNode::MechanamNode(const rclcpp::NodeOptions &options) : Node("er_angle_node", options) {
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
                std::bind(&MechanamNode::_subscriber_callback, this,  std::placeholders::_1));
    }

    void MechanamNode::_subscriber_callback(geometry_msgs::msg::Twist msg) {
        float x_val = msg.linear.x;
        float y_val = msg.linear.y;
        float theta = msg.angular.z;
        float v1 = -1 * x_val + 1 * y_val;
        float v2 = 1 * x_val + 1 * y_val;
        float v3 = -1 * x_val + 1 * y_val;
        float v4 = 1 * x_val + 1 * y_val;

        uint8_t _send_bytes[16];
        uint8_t _tmp_bytes[4];
        convert_float_to_byte(v1, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            _send_bytes[i] = _tmp_bytes[i];
        }
        convert_float_to_byte(v2, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            _send_bytes[i + 4] = _tmp_bytes[i];
        }
        convert_float_to_byte(v3, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            _send_bytes[i + 8] = _tmp_bytes[i];
        }
        convert_float_to_byte(v4, _tmp_bytes);
        for (int i = 0; i < 4; ++i) {
            _send_bytes[i + 12] = _tmp_bytes[i];
        }
        pack_send_bytes(_send_bytes);
    }

    void MechanamNode::_publisher_callback() {
        auto msg = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg->canid = 0x101;
        msg->candlc = 16;
        uint8_t tmp_bytes[16];
        pack_msg_bytes(tmp_bytes);
        for (int i = 0; i < 16; ++i) {
            msg->candata[i] = tmp_bytes[i];
        }
        _publisher->publish(*msg);
    }

    void  MechanamNode::pack_send_bytes(uint8_t _send_bytes[16]) {
        std::lock_guard<std::recursive_mutex> lock(pack_bytes_mtx_);
        for (int i = 0; i < 16; ++i) {
            send_bytes[i] = _send_bytes[i];
        }
    }

    void MechanamNode::pack_msg_bytes(uint8_t (&bytes)[16]) {
        std::lock_guard<std::recursive_mutex> lock(pack_bytes_mtx_);
        for (int i = 0; i < 16; ++i) {
            bytes[i] = send_bytes[i];
        }
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(mechanam_node::MechanamNode)