#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> // Int32 대신 MultiArray 사용
#include "callback.hpp"   // 우리가 깎아 만든 완벽한 뇌
#include "dynamixel.hpp"  // 모터 제어 객체

using namespace Eigen;

class GoatMainNode : public rclcpp::Node 
{
private:
    Callback robot_brain;    //  뇌 객체
    Dxl dxl_port;            //  모터 제어기 객체 (여기서 포트 열리고 토크 켜짐!)
    
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motion_sub_;
    rclcpp::TimerBase::SharedPtr timer_100Hz_;

public:
    GoatMainNode() : Node("goat_main_node") 
    {
        // =========================================================
        // 1. 파이썬 사령부의 명령을 받는 귀 (Subscriber) - 동적 시간 지정을 위해 Float64MultiArray 사용
        // =========================================================
        motion_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/motion_command", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                
                if (msg->data.empty()) {
                    RCLCPP_WARN(this->get_logger(), "빈 모션 명령을 수신하여 무시합니다.");
                    return;
                }

                // 첫 번째 데이터는 모션 번호
                int go_cmd = static_cast<int>(msg->data[0]);
                
                // 두 번째부터 끝까지는 각 구간의 동작 시간(초)
                std::vector<double> durations;
                if (msg->data.size() > 1) {
                    // 벡터의 일부를 복사해서 새로운 벡터를 만듭니다.
                    durations.assign(msg->data.begin() + 1, msg->data.end());
                }

                RCLCPP_INFO(this->get_logger(), "파이썬 명령 수신: %d번 모션 (시간 지정: %zu개)", go_cmd, durations.size());
                
                // 뇌한테 명령 번호와 동적 시간 전달
                robot_brain.SelectMotion(go_cmd, durations); 
            }
        );

        // =========================================================
        // 2. 100Hz 심장 박동 타이머 (진짜 제어 루프)
        // =========================================================
        timer_100Hz_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // 10ms = 100Hz
            [this]() {
                
                // 1. 뇌한테 0.01초짜리 계산 돌리라고 지시 (여기서 배열이 채워짐)
                robot_brain.Write_All_Theta();
                
                // 2. 뇌에서 나온 C++ 일반 배열(double[23])을 Eigen VectorXd로 예쁘게 포장!
                VectorXd target_vector(NUMBER_OF_JOINTS);
                for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
                    target_vector[i] = robot_brain.All_Theta[i];
                }
                
                // 3. 다이나믹셀 클래스에 목표 각도 장전! (여기서 내부적으로 PI 더해줌)
                dxl_port.SetThetaRef(target_vector);
                
                // 4. 모터로 윙~ 치익! 쏴버리기!
                dxl_port.syncWriteTheta();
            }
        );

        RCLCPP_INFO(this->get_logger(), " G.O.A.T 심장 박동 시작! (100Hz 제어 & 토크 ON)");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // 노드 생성 (이때 Dxl 생성자가 불리면서 포트 열고 LED 켜짐!)
    auto node = std::make_shared<GoatMainNode>();
    
    // 영원히 뺑뺑이 (Ctrl+C 누르면 알아서 소멸자 불리고 토크 풀림)
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}