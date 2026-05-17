#pragma once
#include <rclcpp/rclcpp.hpp>
#include <queue> // 🔥 대기표(Queue) 시스템을 위한 C++ 표준 라이브러리 탑재!
#include "sdk.hpp"

// 📝 사령관이 메모장에 적어둘 '주문서' 양식
struct MotionOrder {
    int motion_id;         // 몇 번 모션인지?
    double transition_time; // 몇 초 만에 스무스하게 넘어갈지?
};

class Callback : public rclcpp::Node 
{
private:
    SDK_Motion sdk_engine; 
    
    // 🔥 대기표 관리자 (여기에 주문서가 차곡차곡 쌓입니다)
    std::queue<MotionOrder> motion_queue_;

public:
    Callback();
    
    double All_Theta[NUMBER_OF_JOINTS]; 
    int re; // 0: 대기 중, 1: 모션 재생 중

    void SelectMotion(int go, double transition_time = 1.0);
    void Write_All_Theta();
};