//
// Created by ryuzo on 2023/02/16.
//
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <mutex>

#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "er_angle_node/er_angle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "utilities/can_utils.hpp"

#define pi 3.14159265

namespace er_angle_node {

    ERAngleNode::ERAngleNode(const rclcpp::NodeOptions &options) : Node("er_angle_node", options) {
        using namespace std::chrono_literals;

        declare_parameter("interval_ms", 10);
        interval_ms = this->get_parameter("interval_ms").as_int();
        _publisher = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("/can_tx", _qos);
        _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(interval_ms),
                [this] { _publisher_callback(); }
        );
        _subscription = this->create_subscription<std_msgs::msg::Float32>(
                "/er_angle",
                _qos,
                std::bind(&ERAngleNode::_subscriber_callback, this,  std::placeholders::_1));
        send_flag = false;

        //プログラム起動時に1回だけ実行する関数-----------------------------------------------------------------------------------
        a5_k = -4*l2;
        a4_k_1 = pow(h,2) + 2*(pow(h,2)+pow(l2,2)-pow(l1,2));
        a4_k_2 = 4*pow(l2,2);
        a3_k = -4*l2*(pow(l2,2) + 2*pow(h,2) - pow(l1,2));
        a2_k_1 = pow(pow(l2,2) + pow(h,2) - pow(l1,2),2) + 2*pow(h,2)*(pow(l2,2)+pow(h,2)-pow(l1,2))-4*pow(l2,2)*pow(h,2);
        a2_k_2 = 8*pow(l2,2)*pow(h,2);
        a1_k = -4*l2*pow(h,2)*(pow(l2,2)+pow(h,2)-pow(l1,2));
        a0_k_1 = pow(h,2)*pow(l2,4) + pow(h,6) + pow(h,2)*pow(l1,4) - 2*pow(l2,2)*pow(h,4)-2*pow(l2,2)*pow(l1,2)*pow(h,2)-2*pow(h,4)*pow(l1,2);
        a0_k_2 = 4*pow(l2,2)*pow(h,4);
        //---------------------------------------------------------------------------------------------------------------------
    }

    void ERAngleNode::_subscriber_callback(std_msgs::msg::Float32 msg) {
        float angle = msg.data;
        theta = angle*3.14/180;                //thetaを[rad]単位に変換
        cos_th = cos(theta + alpha + beta);    //変数cos(theta + alpha + beta)を計算

        //thetaに応じて変化するパラメータを再計算
        a6 = 1;
        a5 = a5_k*cos_th;
        a4 = a4_k_2*pow(cos_th,2) + a4_k_1;
        a3 = a3_k*cos_th;
        a2 = a2_k_1 + a2_k_2*pow(cos_th,2);
        a1 = a1_k*cos_th;
        a0 = a0_k_2*pow(cos_th,2) + a0_k_1;

        //ニュートン法
        x = x0 - takoushiki(x0)/bibun(x0);
        while(fabs(x-x0)>0.001){
            x0 = x;
            x = x - takoushiki(x)/bibun(x);
        }

        x = w-x;
        //順運動学により検算
        theta = acos((pow(l2,2) + pow(w-x,2)+pow(h,2)-pow(l1,2))/(2*l2*sqrt(pow(h,2)+pow(w-x,2)))) + atan(h/(w-x)) - alpha - beta;
        theta = theta * 180/3.14;
        send_flag = true;
        convert_float_to_byte(x, send_bytes);
    }

    void ERAngleNode::_publisher_callback() {
        if (send_flag){
            auto msg = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg->canid = 0x301;
            msg->candlc = 4;
            for (int i = 0; i < 4; ++i) {
                msg->candata[i] = send_bytes[i];
            }
            _publisher->publish(*msg);
        }
    }

    float ERAngleNode::takoushiki(float x) {
        float y;
        y = a6*pow(x,6) + a5*pow(x,5) + a4*pow(x,4) + a3*pow(x,3) + a2*pow(x,2) + a1*x + a0;
        return y;
    }

    float ERAngleNode::bibun(float x) {
        float y;
        y = 6*a6*pow(x,5) + 5*a5*pow(x,4) + 4*a4*pow(x,3) + 3*a3*pow(x,2) + 2*a2*x + a1;
        return y;
    }

}

RCLCPP_COMPONENTS_REGISTER_NODE(er_angle_node::ERAngleNode)