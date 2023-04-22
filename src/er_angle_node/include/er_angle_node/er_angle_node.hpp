//
// Created by ryuzo on 2023/02/16.
//

#ifndef BUILD_MECHANAM_NODE_HPP
#define BUILD_MECHANAM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <math.h>

#include "std_msgs/msg/string.hpp"
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace er_angle_node {
    class ERAngleNode final : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _subscription;
        rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _publisher;

        rclcpp::TimerBase::SharedPtr _pub_timer;
        int64_t interval_ms;
        rclcpp::QoS _qos = rclcpp::QoS(5);

        void _subscriber_callback(std_msgs::msg::Float32 msg);
        void _publisher_callback();

        uint8_t send_bytes[4];
        bool send_flag;
        //グローバル変数-----------------------------------------------------------------
//係数を格納する変数の定義
        float a6;
        float a5;
        float a4;
        float a3;
        float a2;
        float a1;
        float a0;

//定数を格納する変数の定義
        float a5_k;
        float a4_k_1;
        float a4_k_2;
        float a3_k;
        float a2_k_1;
        float a2_k_2;
        float a1_k;
        float a0_k_1;
        float a0_k_2;

//機構パラメータの設定
        float l1 = 180.0;       //リンク長さ
        float l2 = 54.83;       //リンク長さ
        float h = 137.5;        //展開機構の回転軸の取付位置
        float w = 189.662938;   //展開機構の回転軸の取付位置
        float alpha = 0.1259;   //展開機構の初期パラメータ[rad]
        float beta = 0.6273;    //展開機構の初期パラメータ[rad]

//逆運動学の入出力に関する変数
        float theta = 60;       //発射機構の目標仰角[deg]
        float cos_th;           //cos_th = cos(theta + alpha + beta)
        float x;                //目標ねじ棒移動量(マイコンに送信する値)
        float x0 = 80;          //ニュートン法初期値
//---------------------------------------------------------------------------------------------------------

//逆運動学計算に用いる多項式を計算する関数の定義------------------------------------------------------------
        float takoushiki(float x);
//-------------------------------------------------------------------------------------------------------

//逆運動学計算に用いる多項式の微分を計算する関数の定義-------------------------------------------------------
        float bibun(float x);
//-----------------------------------------------------------------------------------------------------

    public:
        explicit ERAngleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    };
}


#endif //BUILD_MECHANAM_NODE_HPP
