#include "dynamixel.hpp"
#include <iostream>  

// Dxl 클래스의 생성자에서는 다이나믹셀과의 통신을 설정하고 초기화 작업을 수행합니다.

Dxl::Dxl()
{
    // 1. 벡터 크기 초기화 
    th_.resize(NUMBER_OF_DYNAMIXELS);
    th_last_.resize(NUMBER_OF_DYNAMIXELS);
    th_dot_est_.resize(NUMBER_OF_DYNAMIXELS);
    ref_th_.resize(NUMBER_OF_DYNAMIXELS);
    ref_th_value_.resize(NUMBER_OF_DYNAMIXELS);

    // 2. 초기값 0으로 세팅 (쓰레기값 방지)
    th_.setZero();
    th_last_.setZero();
    th_dot_est_.setZero();
    ref_th_.setZero();
    
    uint8_t dxl_error = 0; // 다이나믹셀 통신 에러 상태를 저장하는 변수입니다. 0이면 성공, 1이면 실패입니다.
    int dxl_comm_result = COMM_TX_FAIL; // 다이나믹셀 통신 결과를 저장하는 변수입니다. COMM_TX_SUCCESS이면 성공, 그 외는 실패입니다.

    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    // Dynamixel SDK의 PortHandler 객체를 생성하여 포트 통신을 관리합니다.
    // DEVICE_NAME은 "/dev/ttyUSB0"로 설정되어 있습니다.
    // getPortHandler 함수는 포트 이름을 입력으로 받아 해당 포트를 관리하는 PortHandler 객체를 반환합니다.

    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    // Dynamixel SDK의 PacketHandler 객체를 생성하여 데이터 패킷 송수신을 처리합니다.
    // PROTOCOL_VERSION은 2.0으로 설정되어 있습니다.
    // getPacketHandler 함수는 프로토콜 버전을 입력으로 받아 해당 버전에 맞는 PacketHandler 객체를 반환합니다.


// ********************포트 열기 및 통신 속도 설정**********************

    if (!portHandler->openPort())
        std::cerr << "[Error] Failed to open the port!" << std::endl;
    else 
        std::cout << "[Info] Succeeded to open the port!" << std::endl;
        // openPort() 함수는 포트를 열려고 시도하며, 성공하면 true를 반환하고 실패하면 false를 반환합니다.
        // 포트 열기에 실패하면 에러 메시지를 출력하고, 성공하면 성공 메시지를 출력합니다.
        // 포트가 성공적으로 열렸는지 확인하는 것은 매우 중요합니다. 포트가 열리지 않으면 다이나믹셀과 통신할 수 없기 때문입니다.

    if (!portHandler->setBaudRate(BAUDRATE))
        std::cerr << "[Error] Failed to set the baudrate!" << std::endl;
    else 
        std::cout << "[Info] Succeeded to set the baudrate!" << std::endl;
        // setBaudRate() 함수는 통신 속도를 설정하려고 시도하며, 성공하면 true를 반환하고 실패하면 false를 반환합니다.
        // 통신 속도 설정에 실패하면 에러 메시지를 출력하고, 성공하면 성공 메시지를 출력합니다.
        // 통신 속도가 올바르게 설정되어야 다이나믹셀과 안정적으로 통신할 수 있습니다. 잘못된 통신 속도는 데이터 손실이나 통신 오류를 초래할 수 있습니다.



// ********************위치 제어 모드로 설정**********************

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++) 
    // 모든 다이나믹셀 모터에 대해 반복합니다.
    // NUMBER_OF_DYNAMIXELS는 23으로 정의되어 있으며, 이는 로봇의 전체 모터 수입니다.
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_OperatingMode, Position_Control_Mode, &dxl_error);
        // 각 다이나믹셀 모터의 제어 모드를 위치 제어 모드로 설정합니다.

        // write1ByteTxRx() 함수는 1바이트 데이터(Tx)를 다이나믹셀에 쓰고 응답(Rx)을 받는 함수입니다.
        // portHandler는 통신을 관리하는 객체입니다.
        // dxl_id[i]는 현재 설정하려는 다이나믹셀 모터의 ID입니다. 1~23으로 설정되어 있습니다.
        // DxlReg_OperatingMode는 제어 모드를 설정하는 레지스터 주소입니다. 주소는 헤더파일에 11로 정의되어 있습니다.
        // Position_Control_Mode는 위치 제어 모드로 설정하는 값입니다. 값은 헤더파일에 1로 정의되어 있습니다.
        // dxl_error는 통신 에러 상태를 저장하는 변수입니다. 
        // &dxl_error로 전달하여 함수 내에서 에러 상태를 업데이트할 수 있도록 합니다.

        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to set position control mode for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] Set position control mode for ID: " << int(dxl_id[i]) << std::endl;
        // write1ByteTxRx() 함수의 반환값이 COMM_SUCCESS가 아니면 통신에 실패한 것이므로 에러 메시지를 출력합니다.
        // COMM_SUCCESS는 다이나믹셀 SDK에서 정의된 상수로, 통신이 성공했을 때 반환되는 값입니다.
        // 통신에 성공하면 성공 메시지를 출력합니다.
    }


// ********************토크 On (모터에 힘 주기)**********************

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to enable torque for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] Torque enabled for ID: " << int(dxl_id[i]) << std::endl;
    }
        // 각 다이나믹셀 모터의 토크를 켜서 모터에 힘을 주도록 설정합니다.
        // 앞이랑 같은 방식으로 write1ByteTxRx() 함수를 사용합니다.
        // DxlReg_TorqueEnable 레지스터에 1을 써서 토크를 켭니다. 주소는 헤더파일에 64로 정의되어 있습니다.


// ********************LED On**********************

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to enable LED for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] LED enabled for ID: " << int(dxl_id[i]) << std::endl;
    }
        // 각 다이나믹셀 모터의 LED를 켭니다. 디버깅용으로 사용됩니다.
        // DxlReg_LED 레지스터에 1을 써서 LED를 켭니다. 주소는 헤더파일에 65로 정의되어 있습니다.


// ********************PID 게인 초기화 (P: 850)**********************

    VectorXd PID_Gain(3); // PID 게인을 저장하는 Eigen의 동적 크기 벡터입니다. 크기는 3으로 설정되어 있습니다 (P, I, D).
    PID_Gain << 850, 0, 0; // P 게인은 850으로 설정하고, I와 D 게인은 0으로 설정합니다. 이는 위치 제어에 필요한 기본적인 PID 설정입니다.
    SetPIDGain(PID_Gain); // PID 게인을 설정하는 함수입니다. PID_Gain 벡터를 입력으로 받아 각 게인을 다이나믹셀에 설정합니다.
}





// Dxl 클래스의 소멸자에서는 프로그램 종료 시 다이나믹셀의 토크를 끄고 포트를 닫는 작업을 수행합니다.
Dxl::~Dxl()
{
    uint8_t dxl_error = 0; // 다이나믹셀 통신 에러 상태를 저장하는 변수입니다. 0이면 성공, 1이면 실패입니다.
    int dxl_comm_result = COMM_TX_FAIL;

    // ********************토크 Off (모터에 힘 풀기)**********************
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_TorqueEnable, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to disable torque for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] Torque disabled for ID: " << int(dxl_id[i]) << std::endl;
    }
    // 각 다이나믹셀 모터의 토크를 꺼서 모터에 힘이 풀리도록 설정합니다.
    // DxlReg_TorqueEnable 레지스터에 0을 써서 토크를 끕니다. 주소는 헤더파일에 64로 정의되어 있습니다.


    // ********************LED Off**********************
    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, dxl_id[i], DxlReg_LED, 0, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
            std::cerr << "[Error] Failed to disable LED for ID: " << int(dxl_id[i]) << std::endl;
        else
            std::cout << "[Info] LED disabled for ID: " << int(dxl_id[i]) << std::endl;
    }
    // 각 다이나믹셀 모터의 LED를 끕니다.
    // DxlReg_LED 레지스터에 0을 써서 LED를 끕니다. 주소는 헤더파일에 65로 정의되어 있습니다.


    portHandler->closePort(); 
    // openPort() 함수를 통해 열었던 포트를 closePort() 함수를 통해 닫습니다. 포트를 닫는 것은 리소스 관리를 위해 매우 중요합니다. 
    // 포트를 닫지 않으면 다른 프로그램에서 해당 포트를 사용할 수 없게 될 수 있습니다.
}


// ************************************ GETTERS 각도 읽기(rad으로 변환)*****************************************



// **********************현재 각도 읽기*************************

void Dxl::syncReadTheta() 
// 헤더파일에서 선언한 syncReadTheta() 함수의 정의입니다. 이 함수는 모든 다이나믹셀 모터의 현재 각도를 읽어서 th_ 벡터에 저장합니다.
// Dxl 클래스의 멤버 함수로, 다이나믹셀 모터의 현재 각도를 읽어오는 역할을 합니다.

{
    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, DxlReg_PresentPosition, 4);
    // dynamixel에 있는 GroupSyncRead 클래스를 사용하여 groupSyncRead 객체를 생성합니다. 
    // 이 객체는 여러 다이나믹셀 모터의 데이터를 동시에 읽어올 수 있도록 도와줍니다.
    // portHandler와 packetHandler는 통신을 관리하는 객체입니다. 
    // DxlReg_PresentPosition은 현재 위치를 읽어오는 레지스터 주소입니다. 주소는 헤더파일에 132로 정의되어 있습니다.
    // 4는 읽어올 데이터의 크기(바이트 수)입니다.

    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) groupSyncRead.addParam(dxl_id[i]);
    // groupSyncRead 객체에 각 다이나믹셀 모터의 ID를 추가합니다.
    // addParam() 함수는 읽어올 다이나믹셀 모터의 ID를 등록하는 함수입니다. 이 함수를 통해 여러 모터의 데이터를 동시에 읽어올 수 있습니다.
    // NUMBER_OF_DYNAMIXELS는 23으로 정의되어 있으며, 이는 로봇의 전체 모터 수입니다. 따라서 1~23번 ID의 모터 데이터를 읽어오도록 설정합니다.
    // dxl_id[i]는 각 모터의 ID가 저장된 배열입니다. 헤더파일에 1~23으로 설정되어 있습니다.

    groupSyncRead.txRxPacket();
    // txRxPacket() 함수는 등록된 모든 모터에 대해 데이터를 읽어오는 명령을 전송하고 응답을 받는 함수입니다.
    // 이 함수를 호출하면 addParam() 함수를 통해 등록된 모든 모터에 대해 현재 위치 데이터를 읽어오는 명령이 전송됩니다.

    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) position[i] = groupSyncRead.getData(dxl_id[i], DxlReg_PresentPosition, 4);
    // addParam() 함수를 통해 등록된 각 모터에 대해 getData() 함수를 호출합니다. 
    // getData() 함수는 특정 모터의 특정 레지스터에서 데이터를 읽어오는 함수입니다. 
    // 132번 레지스터(DxlReg_PresentPosition)에서 4바이트 데이터를 읽어와 position 배열에 저장합니다.
    
    groupSyncRead.clearParam();
    // clearParam() 함수는 addParam() 함수를 통해 등록된 모터 ID 목록을 초기화하는 함수입니다.
    // 제어 주기에 맞춰 데이터를 읽어와야하기 때문에, 다음 제어 주기에서 다시 addParam() 함수를 통해 모터 ID를 등록할 수 있도록 합니다.
    
    for(uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++) th_[i] = convertValue2Radian(position[i]) - PI - zero_manual_offset[i];
    // 마지막으로 읽어온 position 데이터를 convertValue2Radian() 함수를 통해 라디안 단위로 변환하여 th_ 벡터에 저장합니다.
    // th_[i]는 각 모터의 현재 각도를 라디안 단위로 저장하는 벡터입니다. 헤더파일에서 VectorXd th_로 선언되어 있습니다.
    // convertValue2Radian() 함수는 다이나믹셀의 raw 값을 라디안으로 변환하는 함수입니다. RAD_TO_VALUE 상수를 사용하여 변환합니다.
    // 헤더파일에 convertValue2Radian() 함수가 선언되어 있습니다.
}

// 결론적으로 모터의 현재 각도는 th_ 벡터에 라디안 단위로 저장됩니다.
// 함수는 제어 주기에 맞춰 반복적으로 호출되어 모터의 현재 각도를 업데이트하는 역할을 합니다.



VectorXd Dxl::GetThetaAct() // 현재 각도 반환 함수입니다. syncReadTheta() 함수를 통해 th_ 벡터에 저장된 현재 각도를 반환합니다.
{
    syncReadTheta(); // 현재 각도를 읽어서 th_ 벡터에 저장하는 함수입니다. GetThetaAct() 함수가 호출될 때마다 최신 각도 데이터를 읽어와서 반환할 수 있도록 합니다.
    return th_; // th_ 벡터를 반환합니다. th_ 벡터는 각 모터의 현재 각도를 라디안 단위로 저장하는 벡터입니다. 헤더파일에서 VectorXd th_로 선언되어 있습니다.
}

// **********************현재 각속도 읽기**********************************
//Getter() : velocity 읽기 (raw data)
void Dxl::syncReadThetaDot()
{
    dynamixel::GroupSyncRead groupSyncReadThDot(portHandler, packetHandler, DxlReg_PresentVelocity, 4);
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) groupSyncReadThDot.addParam(dxl_id[i]);
    groupSyncReadThDot.txRxPacket();
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++) velocity[i] = groupSyncReadThDot.getData(dxl_id[i], DxlReg_PresentVelocity, 4);
    groupSyncReadThDot.clearParam();
}


// ***********************각속도 변환(rad/s)*************************

//Getter() : 각속도 getter() [rad/s] 
VectorXd Dxl::GetThetaDot()
{
    VectorXd vel_(NUMBER_OF_DYNAMIXELS);
    for(uint8_t i=0; i<NUMBER_OF_DYNAMIXELS; i++)
    {
        if(velocity[i] > 4294900000) vel_[i] = (velocity[i] - 4294967295) * 0.003816667; 
        else vel_[i] = velocity[i] * 0.003816667;
    }
    return vel_;
}

//Getter() : About dynamixel packet data
void Dxl::getParam(int32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

//Getter() : 추정계산 (이전 세타값 - 현재 세타값 / 시간) [rad/s]
void Dxl::CalculateEstimatedThetaDot(int dt_us)
{
    th_dot_est_ = (th_last_ - th_) / (-dt_us * 0.00001);
    th_last_ = th_;
}

//Getter() : 각속도 추정계산 getter() [rad/s] 
VectorXd Dxl::GetThetaDotEstimated()
{
    return th_dot_est_;
}

//Getter() : 현재 모드 getter()
int16_t Dxl::GetPresentMode()
{
    return this->Mode;
}


// **************************** SETTERS ******************************** //




//setter() : 각도 setter() [rad]

void Dxl::syncWriteTheta() 
// 헤더파일에서 선언한 syncWriteTheta() 함수의 정의입니다. 
// 이 함수는 ref_th_ 벡터에 저장된 목표 각도를 다이나믹셀 모터에 전달하여 실제로 모터를 구동하는 역할을 합니다.
{
  dynamixel::GroupSyncWrite gSyncWriteTh(portHandler, packetHandler, DxlReg_GoalPosition, 4);
  // dynamixel에 있는 GroupSyncWrite 클래스를 사용하여 gSyncWriteTh 객체를 생성합니다.
  // 이 객체는 여러 다이나믹셀 모터의 데이터를 동시에 써줄 수 있도록 도와줍니다.
  // portHandler와 packetHandler는 통신을 관리하는 객체입니다.
  // DxlReg_GoalPosition은 목표 위치를 써주는 레지스터 주소입니다. 주소는 헤더파일에 116로 정의되어 있습니다.
  // 4는 써줄 데이터의 크기(바이트 수)입니다.

  uint8_t parameter[4] = {0}; // 4바이트 데이터용 배열로 수정
  // ref_th_ 벡터에 저장된 목표 각도를 라디안에서 다이나믹셀의 raw 값으로 변환하여 ref_th_value_ 벡터에 저장합니다.

  // 벡터 연산(라디안 -> Raw 값 변환)은 반복문 밖에서 한 번만 수행하여 성능 최적화
  ref_th_value_ = ref_th_ * RAD_TO_VALUE; 
  // ref_th_ 벡터의 각 요소에 RAD_TO_VALUE 상수를 곱하여 라디안 단위의 목표 각도를 다이나믹셀의 raw 값으로 변환합니다.
  // RAD_TO_VALUE는 다이나믹셀의 raw 값을 라디안으로 변환하는 상수입니다. 헤더파일에 4096/(2*PI)로 정의되어 있습니다.

  for (uint8_t i=0; i < NUMBER_OF_DYNAMIXELS; i++){
    getParam(ref_th_value_[i], parameter);
    gSyncWriteTh.addParam(dxl_id[i], parameter);
  }
  // getParam() 함수를 사용하여 ref_th_value_ 벡터의 각 요소를 4바이트 데이터로 변환하여 parameter 배열에 저장합니다.
  // addParam() 함수를 통해 각 다이나믹셀 모터의 ID와 변환된 목표 각도 데이터를 gSyncWriteTh 객체에 등록합니다. 
  // 이 함수를 통해 여러 모터의 데이터를 동시에 써줄 수 있습니다.
  // 23개의 모터에 대해 반복하여 각 모터의 ID와 목표 각도 데이터를 등록합니다.

  gSyncWriteTh.txPacket(); // txPacket() 함수는 등록된 모든 모터에 대해 데이터를 써주는 명령을 전송하는 함수입니다.
  gSyncWriteTh.clearParam(); // clearParam() 함수는 addParam() 함수를 통해 등록된 모터 ID와 데이터 목록을 초기화하는 함수입니다.
}





// Setter() : 목표 세타값 설정 [rad]


// SetThetaRef() 함수는 입력으로 받은 theta 벡터를 ref_th_ 벡터에 저장하는 함수입니다.
// 저장된 ref_th_ 벡터는 syncWriteTheta() 함수를 통해 다이나믹셀 모터에 전달되어 실제로 모터를 구동하는 역할을 합니다.
void Dxl::SetThetaRef(VectorXd theta) 
// 입력으로 받은 theta 벡터를 ref_th_ 벡터에 저장하는 함수입니다. 
// theta 벡터는 각 모터의 목표 각도가 -180도에서 180도 사이의 라디안 단위로 저장된 벡터입니다. 헤더파일에서 VectorXd theta로 선언되어 있습니다.
{
    for (uint8_t i=0; i<NUMBER_OF_DYNAMIXELS;i++)  
    {
        ref_th_[i] = theta[i]+PI; 
        // 입력으로 받은 theta 벡터의 각 요소에 PI를 더하여 ref_th_ 벡터에 저장합니다.
        // PI는 180도입니다. 이를 더해주는 이유는 다이나믹셀의 영점이 2048(2바이트로 표현된 중간값)으로 설정되어 있기 때문입니다.
        // PI를 더해주는 이유는 다이나믹셀의 영점이 2048(2바이트로 표현된 중간값)으로 설정되어 있기 때문입니다.
        // 이를 0에서 360도 범위로 변환하기 위해 PI를 더해줍니다. 예를 들어, -180도(-PI)는 0도로, 0도(0)는 180도로, 180도(PI)는 360도로 변환됩니다.
    }
}





// Setter() : PID gain setter()
void Dxl::SetPIDGain(VectorXd PID_Gain)
{    
    uint8_t dxl_error = 0;
    
    if (PID_Gain.size() != 3)
    {
        std::cerr << "PID_Gain should have exactly 3 elements: P, I, and D gains." << std::endl;
        return;
    }
    
    uint16_t P_gain = static_cast<uint16_t>(PID_Gain(0));
    uint16_t I_gain = static_cast<uint16_t>(PID_Gain(1));
    uint16_t D_gain = static_cast<uint16_t>(PID_Gain(2));

    for (uint8_t i = 0; i < NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionPGain, P_gain, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionIGain, I_gain, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, dxl_id[i], DxlReg_PositionDGain, D_gain, &dxl_error);
    }
}

//Setter() : 현재 모드 설정 (위치 제어 모드로 고정)
int16_t Dxl::SetPresentMode(int16_t Mode)
{
    if (Mode == Position_Control_Mode)
    {
        this->Mode = Position_Control_Mode;
        return Position_Control_Mode;
    }
    else
    {
        std::cerr << "[Error] Only Position Control Mode is supported now." << std::endl;
        this->Mode = Position_Control_Mode;
        return Position_Control_Mode;
    }
}

// **************************** Function ******************************** //

//Value2Radian (Raw data -> Radian) 라디안으로 변환하는 함수입니다. 다이나믹셀의 raw 값을 라디안으로 변환하기 위해 RAD_TO_VALUE 상수를 사용합니다.
float Dxl::convertValue2Radian(int32_t value)
{
    float radian = value / RAD_TO_VALUE;
    return radian;
}

//각도(rad), 각속도(rad/s) 읽기 제어 루프
void Dxl::Loop(bool RxTh, bool RxThDot)
{
    if(RxTh) syncReadTheta();
    if(RxThDot) syncReadThetaDot();
}

//dxl 초기 세팅 (오프셋 0으로 초기화) // 다이나믹셀 모터의 영점 오프셋을 0으로 초기화하는 함수입니다. zero_manual_offset 배열을 0으로 채웁니다.
void Dxl::initActuatorValues()
{
    for (int i=0; i<NUMBER_OF_DYNAMIXELS; i++)
        zero_manual_offset[i] = 0;
}

VectorXd Dxl::read_rad() 
// 현재 각도를 직접 읽어서 라디안 단위로 반환하는 함수입니다. syncReadTheta() 함수를 통해 th_ 벡터에 저장된 현재 각도를 반환합니다.
{
    VectorXd rdl_(NUMBER_OF_DYNAMIXELS);
    int32_t present_position = 0;
    for (int i =0; i< NUMBER_OF_DYNAMIXELS; i++)
    {
        packetHandler->read4ByteTxRx(portHandler, dxl_id[i], DxlReg_PresentPosition,(uint32_t*)&present_position);
        rdl_[i] = (present_position - 2048) * (2.0 * M_PI / 4096.0);
    }

    return rdl_;
}

void Dxl::MoveToTargetQuintic(const VectorXd& theta_goal, int steps, int delay_ms) 
{
    // 1. 현재 각도를 읽어옵니다. (출발점)
    VectorXd theta_now = read_rad();

    // 2. steps만큼 쪼개서 중간 목표 지점을 계속 업데이트합니다.
    for (int s = 1; s <= steps; ++s)
    {
        // t는 0.0에서 시작해 1.0으로 끝나는 진행률입니다.
        double t = static_cast<double>(s) / steps;

        // 3. 5차 다항식 (10t^3 - 15t^4 + 6t^5) 연산 최적화
        double t2 = t * t;
        double t3 = t2 * t;
        double rate = t3 * (10.0 - 15.0 * t + 6.0 * t2);

        // 4. 현재 시점(t)에서 가야 할 중간 목표 각도 계산
        VectorXd theta_interp = theta_now + (theta_goal - theta_now) * rate;

        // 5. 모터에 목표 각도 지령 내리기
        SetThetaRef(theta_interp);
        syncWriteTheta();

        // 6. 다음 단계로 넘어가기 전 대기 (이동할 물리적 시간 확보)
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }

    // 7. 루프 종료 후, 미세한 오차를 없애기 위해 최종 목표 각도를 한 번 더 확실하게 쏴줍니다.
    SetThetaRef(theta_goal);
    syncWriteTheta();
    
    // 8. 완전히 멈출 때까지 대기 (연속 동작이 필요하다면 이 시간은 줄이거나 빼도 됩니다.)
    std::this_thread::sleep_for(std::chrono::seconds(3));
}
