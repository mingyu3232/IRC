#include "callback.hpp"

Callback::Callback() : Node("goat_callback_node") 
{
    re = 0;
    for(int i = 0; i < NUMBER_OF_JOINTS; ++i) All_Theta[i] = 0.0;

    sdk_engine.Init_Default_Pose(All_Theta);
    
    RCLCPP_INFO(this->get_logger(), "🧠 G.O.A.T. 뇌 부팅 완료! (📝 예약표 시스템 가동)");
}

// ==========================================================
// 📝 파이썬에서 명령이 날아올 때 (예약 접수처)
// ==========================================================
void Callback::SelectMotion(int go, double transition_time) {
    
    // 1. 만약 로봇이 지금 춤추고(re==1) 있다면?
    if (re == 1) {
        // 🔥 예전처럼 튕겨내지 않고, 대기표를 뽑아서 뒤에 줄을 세웁니다!
        motion_queue_.push({go, transition_time});
        RCLCPP_INFO(this->get_logger(), "📝 춤추는 중! [%d번] 모션 대기표 발급 완료 (현재 대기 인원: %zu명)", go, motion_queue_.size());
        return;
    }

    // 2. 로봇이 가만히 쉬고 있다면? 바로 1번으로 재생 시작!
    bool success = sdk_engine.Generate_Trajectory(go, All_Theta, transition_time);
    
    if (success) {
        re = 1; 
        RCLCPP_INFO(this->get_logger(), "▶️ [%d번] 모션 즉시 재생 시작!", go);
    } else {
        RCLCPP_ERROR(this->get_logger(), "❌ [%d번] 모션이 도서관에 없습니다!", go);
    }
}

// ==========================================================
// ⏱️ 100Hz 심장 박동 (논스톱 콤보 재생기)
// ==========================================================
void Callback::Write_All_Theta() {
    
    // 로봇이 춤을 추고 있을 때만 실행
    if (re == 1) {
        bool is_playing = sdk_engine.Get_Next_Tick(All_Theta);
        
        // 🚨 방금 재생하던 모션이 끝나는 '0.01초 찰나의 순간' !!!
        if (!is_playing) {
            
            // 1. 메모장(대기표)에 다음 손님이 남아있는지 확인!
            if (!motion_queue_.empty()) {
                
                // 대기표 1번 손님 호출!
                MotionOrder next_order = motion_queue_.front();
                motion_queue_.pop(); // 불렀으니 대기열에서 지움

                // 🔥 현재 멈춘 자세(All_Theta)에서 '딜레이 없이' 곧바로 다음 모션 궤적 굽기!
                bool success = sdk_engine.Generate_Trajectory(next_order.motion_id, All_Theta, next_order.transition_time);
                
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "🔄 논스톱 콤보 발동! 대기표 1번 [%d번] 모션 이어서 재생합니다!", next_order.motion_id);
                    // re = 1 상태를 그대로 유지하니까 로봇은 멈추지 않고 춤을 이어나감!
                } else {
                    re = 0; // 에러 나면 어쩔 수 없이 정지
                    RCLCPP_ERROR(this->get_logger(), "❌ 대기열의 [%d번] 모션이 없습니다!");
                }
            } 
            // 2. 대기표에 아무도 없으면 깔끔하게 휴식
            else {
                re = 0; 
                RCLCPP_INFO(this->get_logger(), "✅ 모든 예약된 모션 재생 완료! 멈춰서 대기합니다.");
            }
        }
    }
}