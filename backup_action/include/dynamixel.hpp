// 다이나믹셀 제어 클래스 정의 (위치 제어 전용 다이어트 버전)

#ifndef DYNAMIXEL_HPP 
#define DYNAMIXEL_HPP 

#include <iostream> 
#include <vector> 
#include <cmath> 
#include <thread> 
#include <chrono> 
#include <Eigen/Dense> 
#include "dynamixel_sdk/dynamixel_sdk.h" 

using namespace Eigen; // (수정됨: 불필요한 특수기호 제거)

// **************************** 다이나믹셀 제어 상수 **************************** //
#define DEVICE_NAME         "/dev/ttyUSB0" // 포트 이름
#define BAUDRATE            2000000        // 통신 속도
#define PROTOCOL_VERSION    2.0            // 프로토콜 버전
#define NUMBER_OF_DYNAMIXELS 22         // G.O.A.T 전체 관절 수

// 제어 모드 상수
#define Position_Control_Mode 3 // 위치 제어 모드 (X-시리즈 기준 3이 위치 제어 모드입니다)

// 다이나믹셀 레지스터 주소 (X-시리즈 기준) 
#define DxlReg_OperatingMode  11 // 제어 모드 설정 레지스터. 위치 제어 모드로 설정할 때 사용합니다.
#define DxlReg_TorqueEnable   64 // 토크 온/오프 레지스터
#define DxlReg_LED            65 // LED 제어 레지스터 (디버깅용)
#define DxlReg_PositionDGain  80 // 위치 D 게인 레지스터
#define DxlReg_PositionIGain  82 // 위치 I 게인 레지스터
#define DxlReg_PositionPGain  84 // 위치 P 게인 레지스터
#define DxlReg_GoalPosition   116 // 목표 위치 레지스터
#define DxlReg_PresentVelocity 128 // 현재 속도 레지스터
#define DxlReg_PresentPosition 132 // 현재 위치 레지스터


// 변환 계수
#define RAD_TO_VALUE          (4096.0 / (2.0 * M_PI)) // 라디안을 다이나믹셀의 raw 값으로 변환하는 계수 (X-시리즈 기준)

class Dxl // 다이나믹셀 제어 클래스
{
private:
    // SDK 관련 핸들러
    dynamixel::PortHandler *portHandler; // dynamixel 소속의 포트 핸들러
    dynamixel::PacketHandler *packetHandler; // dynamixel 소속의 패킷 핸들러
    // PortHandler와 PacketHandler는 Dynamixel SDK에서 제공하는 클래스입니다.
    // PortHandler는 시리얼 포트 통신을 관리하고, PacketHandler는 다이나믹셀과의 데이터 패킷 송수신을 처리합니다
    // 너무 무거운 객체이므로 포인터로 관리합니다.

    // 하드웨어 정보
    uint8_t dxl_id[NUMBER_OF_DYNAMIXELS] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22 };
    int16_t Mode = Position_Control_Mode; // 제어 모드 (위치 제어 모드로 고정)

    // 데이터 저장용 변수 (Raw & Radian)
    int32_t position[NUMBER_OF_DYNAMIXELS];
    int32_t velocity[NUMBER_OF_DYNAMIXELS];

    // VectorXd는 Eigen 라이브러리에서 제공하는 동적 크기의 벡터 타입입니다. 
    // 각도와 속도 데이터를 저장하는 데 사용됩니다.
    // 모터의 값들을 배열로 받아와 Eigen의 VectorXd로 변환하여 저장합니다.


    double zero_manual_offset[NUMBER_OF_DYNAMIXELS];
    // 다이나믹셀 모터 내부의 영점 2048을 모터 좌표계의 영점으로 맞추기 위한 수동 오프셋 배열입니다.
    // 모터의 영점이 실제로 2048이 아닐 수 있기 때문에
    // 이 배열을 통해 각 모터의 영점 오프셋을 수동으로 조정할 수 있습니다.

    const double PI = 3.141592653589793; // 원주율 상수

public:
    Dxl();  // 생성자: 포트 열기 및 초기 설정
    ~Dxl(); // 소멸자: 토크 끄기 및 포트 닫기

    VectorXd th_;           // 현재 각도 [rad]
    VectorXd th_last_;      // 이전 각도 (속도 추정용)
    VectorXd th_dot_est_;   // 추정된 각속도 [rad/s]
    
    VectorXd ref_th_;       // 목표 각도 [rad]
    VectorXd ref_th_value_; // 목표 각도 [raw]
    
    // **************************** GETTERS ******************************** //
    void syncReadTheta();           // 모든 모터 각도 동시 읽기
    VectorXd GetThetaAct();         // 현재 각도 반환
    void syncReadThetaDot();        // 모든 모터 속도 동시 읽기
    VectorXd GetThetaDot();         // 현재 각속도 반환
    void CalculateEstimatedThetaDot(int dt_us); // 속도 추정 계산
    VectorXd GetThetaDotEstimated(); // 추정 속도 반환
    int16_t GetPresentMode();       // 현재 제어 모드 반환
    VectorXd read_rad();            // 현재 각도 직접 읽기

    // **************************** SETTERS ******************************** //
    void syncWriteTheta();          // 목표 각도 동시 전송
    void SetThetaRef(VectorXd theta); // 목표 각도 설정
    void SetPIDGain(VectorXd PID_Gain);   // PID 게인 설정
    int16_t SetPresentMode(int16_t Mode); // 제어 모드 변경 (전류/위치)

    // **************************** FUNCTIONS ****************************** //
    void getParam(int32_t data, uint8_t *param); // 패키지 데이터 변환
    float convertValue2Radian(int32_t value); // Raw -> Radian 변환
    void Loop(bool RxTh, bool RxThDot); // 제어 루프 (TxTorque 매개변수 삭제)
    void initActuatorValues(); // 액추에이터 초기화
    void MoveToTargetSmoothCos(const VectorXd& theta_goal, int steps, int delay_ms); // 부드러운 이동 (S-Curve)
    void MoveToTargetQuintic(const VectorXd& theta_goal, int steps, int delay_ms); // 5차 다항식 보간 이동
};

#endif // DYNAMIXEL_HPP