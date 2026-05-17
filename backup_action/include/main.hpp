#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Dense>

// 우리가 깎아 만든 뇌와 근육 헤더 포함!
#include "callback.hpp"
#include "dynamixel.hpp"

// GoatMainNode는 통신과 타이머를 관리하는 '심장' 역할을 합니다.
class GoatMainNode : public rclcpp::Node 
{
private:
    // 🧠 뇌 객체 (계산 전담)
    Callback robot_brain; 
    
    // 🦾 모터 제어기 객체 (다이나믹셀 통신 전담)
    Dxl dxl_port;            
    
    // 📡 ROS 2 통신망 객체들
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motion_sub_;
    rclcpp::TimerBase::SharedPtr timer_100Hz_;

public:
    // 생성자 (여기서 토픽 구독과 100Hz 타이머가 세팅됨)
    GoatMainNode();
};