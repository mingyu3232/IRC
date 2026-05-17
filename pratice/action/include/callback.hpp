#pragma once
#include <rclcpp/rclcpp.hpp>
#include "sdk.hpp"

class Callback : public rclcpp::Node 
{
private:
    SDK_Motion sdk_engine; 

public:
    Callback();
    
    double All_Theta[NUMBER_OF_JOINTS]; 

    void SelectMotion(int go, double transition_time = 0.5);
    void Write_All_Theta();

    // 🚀 로봇이 현재 움직이는 중인지(Active 슬롯에 궤적이 있는지) 확인하는 함수
    bool IsMoving();
};