#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include "callback.hpp"
#include "dynamixel.hpp"

using namespace Eigen;

// 🚀 키보드 입력을 비차단(Non-blocking)으로 받기 위한 함수
int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

class GoatMainNode : public rclcpp::Node 
{
private:
    Callback robot_brain;    
    Dxl dxl_port;            
    
    rclcpp::TimerBase::SharedPtr timer_100Hz_;
    std::thread keyboard_thread_;

    bool is_virtual_; 

public:
    GoatMainNode(bool use_virtual = true) : Node("goat_main_node"), dxl_port(use_virtual), is_virtual_(use_virtual) 
    {
        // =========================================================
        // 🚀 [안정화 로직] 초기 데이터 신뢰성 검증 (강화 버전)
        // =========================================================
        RCLCPP_INFO(this->get_logger(), "🔄 초기 데이터 신뢰성 검증 중...");
        
        VectorXd validated_theta(NUMBER_OF_JOINTS);
        validated_theta.setZero();

        if (!is_virtual_) {
            int valid_count = 0;
            for(int k=0; k<10; ++k) { // 최대 10번 시도
                VectorXd cur_read = dxl_port.GetThetaAct();
                
                // 모든 모터 값이 0.0이 아닌지 확인 (통신 에러 방지)
                bool is_valid = true;
                if (cur_read.norm() < 0.001) is_valid = false;

                if (is_valid) {
                    validated_theta += cur_read;
                    valid_count++;
                }
                
                if (valid_count >= 3) break; // 유효 데이터 3개 확보 시 종료
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (valid_count > 0) {
                validated_theta /= (double)valid_count;
            } else {
                RCLCPP_ERROR(this->get_logger(), "현재 관절각을 읽지 못했습니다. 안전을 위해 모션을 시작하지 않습니다.");
                rclcpp::shutdown();
                return;
            }
        } else {
            validated_theta = dxl_port.GetThetaAct();
        }
        
        // 뇌(robot_brain)의 현재 각도를 읽어온 실제 값으로 설정 (초기 동기화)
        robot_brain.SetCurrentTheta(validated_theta);

        if (is_virtual_) {
            RCLCPP_WARN(this->get_logger(), "🤖 [VIRTUAL] 가상 모드 실행 중... (모터 통신 안 함)");
        } else {
            RCLCPP_INFO(this->get_logger(), "🔥 [REAL] 실제 모터 모드 가동 완료!");
        }

        // =========================================================
        // 1. 안전한 시작: 기본 자세(0번)로 이동
        // =========================================================
        RCLCPP_INFO(this->get_logger(), "🐢 안전 모드: 기본 자세로 이동합니다.");
        robot_brain.SelectMotion(0);  // 0번 모션, SDK에 설정된 3초 전환 (안전하게 시작)
        // SelectMotion 함수는 모션 ID를 입력받아서, SDK에 설정된 시간 동안 해당 모션으로 부드럽게 전환하는 기능을 수행합니다.
        // all_theta 배열을 업데이트하여, 3초 동안 0번 모션의 궤적을 따라가도록 설정합니다. 이 과정에서 로봇이 갑자기 움직이지 않도록 안전하게 시작할 수 있습니다.
        // all_theta가 채워지면 바로 실행되는게 아니지 않나? 따로 실행해주는 함수가 있어야 3초동안 실행되는 거 아닌가?
        // → SelectMotion 함수 내부에서 모션 시퀀스가 생성되고, Get_Next_Tick() 함수가 100Hz 제어 루프에서 호출될 때마다 현재 재생 중인 궤적의 다음 프레임에 해당하는 각도들이 계산되어 all_theta 배열에 업데이트됩니다.
        // 따라서, SelectMotion 함수를 호출하는 것만으로도 3초 동안 0번 모션의 궤적이 생성되고, 제어 루프에서 자동으로 실행되도록 설정됩니다. 별도의 실행 함수는 필요하지 않습니다.
        // 제어루프는 100Hz로 계속 돌면서 Get_Next_Tick() 함수를 호출하여 all_theta를 업데이트하기 때문에, SelectMotion 함수에서 모션 시퀀스를 생성하는 것만으로도 해당 모션이 제어 루프에서 실행되도록 보장됩니다.

         // =========================================================
        // =========================================================
        // 2. 키보드 입력 감지 스레드 (테스트용)
        // =========================================================
        keyboard_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "⌨️ 키보드 테스트 모드 가동! (1: 공줍기, 0: 기본자세)");
            while (rclcpp::ok()) {
                int key = getch(); 
                if (key == '1') {
                    RCLCPP_INFO(this->get_logger(), "PRESSED: 1 (명령 전송)");
                    robot_brain.SelectMotion(1); // 1번 모션, SDK에 설정된 시간으로 전환
                } else if (key == '0') {
                    RCLCPP_INFO(this->get_logger(), "PRESSED: 0 (명령 전송)");
                    robot_brain.SelectMotion(0); // 0번 모션, SDK에 설정된 시간으로 전환
                } else if (key == 'q') {
                    RCLCPP_INFO(this->get_logger(), "테스트 종료...");
                    rclcpp::shutdown();
                    break;
                }
            }
        });

        // =========================================================
        // 3. 100Hz 제어 루프 (실제 동작)
        // =========================================================
        timer_100Hz_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() {
                // 1. 뇌 연산 (0.01초 분량 각도 계산)
                robot_brain.Write_All_Theta();
                
                // 2. 데이터 변환 및 모터 전송
                VectorXd target_vector(NUMBER_OF_JOINTS);
                for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
                    target_vector[i] = robot_brain.All_Theta[i];
                }
                dxl_port.SetThetaRef(target_vector);
                dxl_port.syncWriteTheta();

                // 🚀 [디버깅] 로봇이 움직이는 중일 때만 현재 상태를 출력
                if (robot_brain.IsMoving()) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "로봇 동작 중...");
                }
            }
        );

        RCLCPP_INFO(this->get_logger(), " G.O.A.T 심장 박동 시작!");
    }

    ~GoatMainNode() {
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    // 🚀 실제 모드로 돌리려면 아래를 false로 설정하세요!
    auto node = std::make_shared<GoatMainNode>(false); 
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
