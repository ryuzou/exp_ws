//
// Created by ryuzo on 2023/02/16.
//

#ifndef BUILD_MECHANAM_NODE_HPP
#define BUILD_MECHANAM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "std_msgs/msg/string.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace mechanam_node {
    class MechanamNode final : public rclcpp::Node {
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _subscription;
        rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher;

        rclcpp::TimerBase::SharedPtr _pub_timer;
        int64_t interval_ms;
        rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

        void _subscriber_callback(geometry_msgs::msg::Twist msg);
        void _publisher_callback();

        uint8_t send_bytes[16];

        std::recursive_mutex pack_bytes_mtx_;
        void pack_send_bytes(uint8_t _send_bytes[16]);
        void pack_msg_bytes(uint8_t (&bytes)[16]);

    public:
        explicit MechanamNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    };
}


#endif //BUILD_MECHANAM_NODE_HPP
