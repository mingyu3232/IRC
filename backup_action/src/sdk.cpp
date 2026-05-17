#include "sdk.hpp"
#include <cmath>
#include <iostream>


SDK_Motion::SDK_Motion() // 생성자 생성
{
    current_seg_idx_ = 0; // 변수 초기화, 0번 구간부터 실행
    current_tick_ = 0; // 변수 초기화, 0번 프레임 부터 시작
    define_motions(); 
    // motion_libaray를 map으로 정의했었는데 , 
    // key인 모션 번호와 Value인 MotionSequence (그 모션의 실제 22개 모터 각도)를 RAM(map)에 입력시킴
    
    // motion_library_[0] = {기본 자세 데이터 뭉치}; 
    // motion_library_[1] = {공 줍기 데이터 뭉치}; 
}


void SDK_Motion::Init_Default_Pose(double* All_Theta) 
// 부팅 전용 함수이므로 0번(대기자세)만 찾음.
// 나머지 모션들은 모션 실행 시 검사함
{
    if (motion_library_.count(0)) // map 안에 0 이라는 key 존재하는지 확인 > 쓰레기 객체 방지
    {
        for(int i = 0; i < NUMBER_OF_JOINTS; ++i) 
        {
            All_Theta[i] = motion_library_[0].poses[0][i]; 
            // All Theta에 momotion_library_0번 (대기자세)의 0번 poses의 모터값 채워넣기 (대기자세는 1개 포즈밖에 없긴 함)
        }
    }
}



void SDK_Motion::define_motions() 
// 프로그램 부팅 시 단 한 번 실행되어, 하드코딩된 모터 제어 데이터들을 라디안으로 변환한 뒤 RAM에 Load하는 데이터 초기화 함수
{
    auto R = [](int raw_value) { return (raw_value - 2048.0) * (M_PI / 2048.0); };
    // 익명함수 > R(2082)라고 감싸기만 하면 프로그램이 켜질 때 CPU가 알아서 라디안으로 변환해서 메모리에 저장하도록 자동화


    // [0번: 기본 자세]
    MotionSequence pose_default; 
    // 변수 선언 >>  MotionSequence는 헤더 파일에 정의된 구조체(자료형이지만 흰색으로 뜸), pose_default는 변수이름 
    pose_default.poses = {{R(2082), R(1440), R(2187), R(2242), R(2082), R(2056), R(2682), R(1764), R(1456), R(2018), R(2069), R(2108), R(2042), R(1630), R(999), R(1957), R(2088), R(2439), R(3108), R(2149), R(1993), R(1964)}};
    // pose는 2차원 벡터이므로 형식에 맞게 저장 >> {{}} 괄호가 2개임. pose가 1개라 시간 지정도 필요없음.
    motion_library_[0] = pose_default; // pose_default 모션을 motion_library_ 0번 슬롯에 넣기


    // [1번: 공 줍기]
    MotionSequence motion_pickup;
    motion_pickup.poses = 
    {
        {R(2099), R(780), R(3006), R(2412), R(2092), R(2049), R(3390), R(949), R(1343), R(1990), R(2045), R(2106), R(2064), R(1631), R(1001), R(1959), R(2089), R(2438), R(3105), R(2148), R(1993), R(1963)},
        {R(2100), R(784), R(3023), R(2405), R(2100), R(2046), R(3397), R(943), R(1347), R(1989), R(2051), R(2078), R(2059), R(1631), R(1005), R(1694), R(2093), R(2437), R(3101), R(2417), R(1948), R(1963)},
        {R(2100), R(784), R(3023), R(2405), R(2100), R(2046), R(3397), R(943), R(1347), R(1989), R(2051), R(1209), R(2059), R(1023), R(1764), R(1793), R(2081), R(2437), R(3101), R(2417), R(1948), R(1963)},
        {R(2116), R(560), R(3245), R(2433), R(2126), R(2007), R(3559), R(729), R(1269), R(1999), R(2074), R(1210), R(2051), R(1023), R(1767), R(1796), R(2082), R(2437), R(3099), R(2414), R(1946), R(1964)},
        {R(2115), R(561), R(3246), R(2433), R(2128), R(2006), R(3559), R(728), R(1269), R(2001), R(2075), R(1205), R(2051), R(1458), R(1931), R(1826), R(2033), R(2437), R(3097), R(2410), R(1946), R(1964)},
        {R(2115), R(562), R(3246), R(2435), R(2130), R(2008), R(3560), R(728), R(1269), R(2003), R(2077), R(1208), R(2052), R(1446), R(895), R(1683), R(2149), R(2437), R(3093), R(2404), R(1944), R(1964)}
    }; // pose는 2차원 벡터이므로 형식에 맞게 저장
    motion_pickup.durations = {1.0, 1.0, 1.0, 1.0, 1.0}; // 구간별 1.0초 지정, 포즈가 6개 이므로 경유지는 5개
    motion_library_[1] = motion_pickup; // motion_pickup 모션을 motion_library_ 1번 슬롯에 넣기




    // ***************************여기에 원하는 모션을 위 형식에 맞춰 넣습니다 !!!!!******************************


}


// [수학 공식] 5차 다항식 계수 계산
void SDK_Motion::calculate_coefficients //이 함수는 시작점과 끝점의 상태(위치, 속도, 가속도)를 입력받아서
// 두 점 사이를 부드럽게 잇는 5차 다항식의 계수 6개를 모터 22개 분량만큼 계산해 메모리에 기록하는 함수입니다.
(
    const std::vector<double>& q0, const std::vector<double>& qf, // 시작/끝 위치 (q)
    const std::vector<double>& v0, const std::vector<double>& vf, // 시작/끝 속도 (v)
    const std::vector<double>& a0, const std::vector<double>& af, // 시작/끝 가속도 (a)
    // const &(상수 참조자): 배열을 복사해 오면 메모리와 시간이 낭비되니, 원본 메모리 주소만 읽기 전용(const)으로 참조해서 가져옵니다.
    double T, // 도달 시간
    // q와 같이 배열 하나에 22개의 8바이트 double이 들어있지 않아 값 자체를 복사해서(Call by Value) 넘기는 게 CPU 캐시 메모리 구조상 훨씬 빠릅니다.
    TrajectorySegment& segment
    // 계산된 결과를 담을 빈 구조체(티켓)의 메모리 주소입니다. 
    // TrajectorySegment 구조체 타입으로 만든 segment 변수입니다. 
    // 이 함수가 종료되면 이 주소에 132개(22 x 6)의 5차 다항식 계수가 꽉 채워집니다.
)
{
    segment.c0.resize(NUMBER_OF_JOINTS); segment.c1.resize(NUMBER_OF_JOINTS); segment.c2.resize(NUMBER_OF_JOINTS);
    segment.c3.resize(NUMBER_OF_JOINTS); segment.c4.resize(NUMBER_OF_JOINTS); segment.c5.resize(NUMBER_OF_JOINTS);
    double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        double dq = qf[i] - q0[i];
        segment.c0[i] = q0[i]; segment.c1[i] = v0[i]; segment.c2[i] = a0[i] / 2.0;
        segment.c3[i] = (10*dq - (6*v0[i] + 4*vf[i])*T - (1.5*a0[i] - 0.5*af[i])*T2) / T3;
        segment.c4[i] = (-15*dq + (8*v0[i] + 7*vf[i])*T + (1.5*a0[i] - af[i])*T2) / T4;
        segment.c5[i] = ( 6*dq - 3*(v0[i] + vf[i])*T - (0.5*a0[i] - 0.5*af[i])*T2) / T5;
    }
    segment.total_ticks = static_cast<int>(T * HZ);
}



// 🔥 [궤적 장전] V, A 계산 후 스플라인 굽기
bool SDK_Motion::Generate_Trajectory(int motion_id, double* current_pose, double transition_time) {
    if (motion_library_.find(motion_id) == motion_library_.end()) return false;

    planned_trajectory_.clear();
    const auto& seq = motion_library_.at(motion_id);
    
    std::vector<std::vector<double>> waypoints;
    waypoints.push_back(std::vector<double>(current_pose, current_pose + NUMBER_OF_JOINTS));
    for (const auto& p : seq.poses) waypoints.push_back(p);

    std::vector<double> durations;
    
    // 1.0 고정이었던 자리에 파라미터로 받은 transition_time 을 쏙!
    durations.push_back(transition_time); 
    
    for (double d : seq.durations) durations.push_back(d);

    int M = waypoints.size();
    std::vector<std::vector<double>> V(M, std::vector<double>(NUMBER_OF_JOINTS, 0.0));
    std::vector<std::vector<double>> A(M, std::vector<double>(NUMBER_OF_JOINTS, 0.0));

    for (int i = 1; i < M - 1; ++i) {
        for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
            V[i][j] = (waypoints[i+1][j] - waypoints[i-1][j]) / (durations[i-1] + durations[i]);
        }
    }
    for (int i = 1; i < M - 1; ++i) {
        for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
            A[i][j] = (V[i+1][j] - V[i-1][j]) / (durations[i-1] + durations[i]);
        }
    }

    for (int i = 0; i < M - 1; ++i) {
        TrajectorySegment seg;
        calculate_coefficients(waypoints[i], waypoints[i+1], V[i], V[i+1], A[i], A[i+1], durations[i], seg);
        planned_trajectory_.push_back(seg);
    }

    current_seg_idx_ = 0;
    current_tick_ = 0;
    return true;
}

// [실시간 재생기] Callback이 부르면 0.01초 분량 계산해서 뱉음
bool SDK_Motion::Get_Next_Tick(double* All_Theta) {
    if (current_seg_idx_ >= planned_trajectory_.size()) return false; // 끝남!

    const auto& seg = planned_trajectory_[current_seg_idx_];
    double t = static_cast<double>(current_tick_) / HZ;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
        All_Theta[i] = seg.c0[i] + seg.c1[i]*t + seg.c2[i]*t2 + seg.c3[i]*t3 + seg.c4[i]*t4 + seg.c5[i]*t5;
    }

    current_tick_++;
    if (current_tick_ >= seg.total_ticks) {
        current_seg_idx_++;
        current_tick_ = 0;
    }
    return true; // 아직 재생 중!
}