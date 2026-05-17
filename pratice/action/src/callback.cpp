#include "callback.hpp"

Callback::Callback() : Node("goat_callback_node") 
{
    // 초기 관절각 0으로 세팅
    // 실제 시작 각도는 main.cpp에서 모터 값을 읽어와서 덮어씌울 예정입니다.
    for(int i = 0; i < NUMBER_OF_JOINTS; ++i) All_Theta[i] = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "🧠 G.O.A.T. 뇌 최적화 완료! (내장형 듀얼 버퍼 시스템 가동)");
}

// ==========================================================
// 🚀 명령 수신 (SDK의 지능형 예약 시스템 활용)
// ==========================================================
void Callback::SelectMotion(int go, double transition_time) {
    // SDK 내부에서 현재 상태를 체크하여 [즉시 실행] 또는 [Pending 예약]을 알아서 판단함
    bool success = sdk_engine.Generate_Trajectory(go, All_Theta, transition_time);
    
    if (success) {
        RCLCPP_INFO(this->get_logger(), "📥 [%d번] 모션 접수 완료 (연속성 확보됨)", go);
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ [%d번] 모션을 찾을 수 없습니다!", go);
    }
}

// ==========================================================
// ⏱️ 100Hz 심장 박동 (SDK 데이터를 배열에 쓰기)
// ==========================================================
void Callback::Write_All_Theta() {
    // SDK가 0.01초 분량의 데이터를 계산해서 All_Theta 배열을 채워줌
    sdk_engine.Get_Next_Tick(All_Theta);
}

// ==========================================================
// 🚀 상태 확인 (현재 움직이는 중인가?)
// ==========================================================
bool Callback::IsMoving() {
    // 로봇이 현재 움직이는 중인지(Active 슬롯에 궤적이 있는지) 확인하는 함수
    return sdk_engine.Is_Moving();
}