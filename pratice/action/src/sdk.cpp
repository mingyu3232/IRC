#include "sdk.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

// 각도 차이를 계산할 때만 사용하는 헬퍼 함수
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

SDK_Motion::SDK_Motion() 
{
    current_seg_idx_ = 0; 
    current_tick_ = 0; 
    current_motion_id_ = -1; 
    pending_motion_id_ = -1;
    current_v_.assign(NUMBER_OF_JOINTS, 0.0);
    current_a_.assign(NUMBER_OF_JOINTS, 0.0);
    define_motions(); 
}

void SDK_Motion::Init_Default_Pose(double* All_Theta) 
{
    if (motion_library_.count(0)) 
    {
        for(int i = 0; i < NUMBER_OF_JOINTS; ++i) 
        {
            All_Theta[i] = motion_library_[0].poses[0][i]; 
        }
    }
}

void SDK_Motion::define_motions() 
{
    auto R = [](int raw_value) { return (raw_value - 2048.0) * (M_PI / 2048.0); };

    MotionSequence pose_default; 
    pose_default.poses = {{R(2075), R(1372), R(2225), R(2223), R(2054), R(2069), R(2753), R(1729), R(1465), R(2043), R(2045), R(2110), R(2058), R(1513), R(1007), R(1960), R(2093), R(2438), R(3093), R(2148), R(1989), R(1963)}};
    motion_library_[0] = pose_default; 

    MotionSequence motion_pickup;
    motion_pickup.poses = 
    {
        {R(2099), R(780), R(3006), R(2412), R(2092), R(2049), R(3390), R(949), R(1343), R(1990), R(2045), R(2106), R(2064), R(1631), R(1001), R(1959), R(2089), R(2438), R(3105), R(2148), R(1993), R(1963)},
        {R(2100), R(784), R(3023), R(2405), R(2100), R(2046), R(3397), R(943), R(1347), R(1989), R(2051), R(2078), R(2059), R(1631), R(1005), R(1694), R(2093), R(2437), R(3101), R(2417), R(1948), R(1963)},
        {R(2100), R(784), R(3023), R(2405), R(2100), R(2046), R(3397), R(943), R(1347), R(1989), R(2051), R(1209), R(2059), R(1023), R(1764), R(1793), R(2081), R(2437), R(3101), R(2417), R(1948), R(1963)},
        {R(2116), R(560), R(3245), R(2433), R(2126), R(2007), R(3559), R(729), R(1269), R(1999), R(2074), R(1210), R(2051), R(1023), R(1767), R(1796), R(2082), R(2437), R(3099), R(2414), R(1946), R(1964)},
        {R(2115), R(561), R(3246), R(2433), R(2128), R(2006), R(3559), R(728), R(1269), R(2001), R(2075), R(1205), R(2051), R(1458), R(1931), R(1826), R(2033), R(2437), R(3097), R(2410), R(1946), R(1964)},
        {R(2115), R(562), R(3246), R(2435), R(2130), R(2008), R(3560), R(728), R(1269), R(2003), R(2077), R(1208), R(2052), R(1446), R(895), R(1683), R(2149), R(2437), R(3093), R(2404), R(1944), R(1964)}
    };
    motion_pickup.durations = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; 
    motion_library_[1] = motion_pickup; 
}

void SDK_Motion::calculate_coefficients 
(
    const std::vector<double>& q0, const std::vector<double>& qf, 
    const std::vector<double>& v0, const std::vector<double>& vf, 
    const std::vector<double>& a0, const std::vector<double>& af, 
    double T, TrajectorySegment& segment
)
{
    segment.c0.resize(NUMBER_OF_JOINTS); segment.c1.resize(NUMBER_OF_JOINTS); segment.c2.resize(NUMBER_OF_JOINTS);
    segment.c3.resize(NUMBER_OF_JOINTS); segment.c4.resize(NUMBER_OF_JOINTS); segment.c5.resize(NUMBER_OF_JOINTS);
    double T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        double dq = normalize_angle(qf[i] - q0[i]); 
        segment.c0[i] = q0[i]; segment.c1[i] = v0[i]; segment.c2[i] = a0[i] / 2.0;
        segment.c3[i] = (10*dq - (6*v0[i] + 4*vf[i])*T - (1.5*a0[i] - 0.5*af[i])*T2) / T3;
        segment.c4[i] = (-15*dq + (8*v0[i] + 7*vf[i])*T + (1.5*a0[i] - af[i])*T2) / T4;
        segment.c5[i] = ( 6*dq - 3*(v0[i] + vf[i])*T - (0.5*a0[i] - 0.5*af[i])*T2) / T5;
    }
    segment.total_ticks = static_cast<int>(T * HZ);
}

bool SDK_Motion::Generate_Trajectory(int motion_id, double* current_pose, double transition_time) {
    if (motion_library_.find(motion_id) == motion_library_.end()) return false;

    if (Is_Moving()) {
        int last_idx = (int)active_trajectory_.size() - 1;
        
        // 🔒 [1. 마지막 구간 차단]
        if (current_seg_idx_ >= last_idx) {
            std::cerr << "[SDK] 예약 거부: 이미 마지막 구간입니다." << std::endl;
            return false;
        }

        // 🔒 [2. 80% Lock-out] 현재 구간이 거의 끝나갈 때는 재계산 안 함 (물리적 안정성)
        double progress = (double)current_tick_ / active_trajectory_[current_seg_idx_].total_ticks;
        if (progress > 0.8) {
            std::cout << "[SDK] 예약 잠금: 구간 진행률 " << (int)(progress*100) << "% - 다음 구간 시작 시까지 대기" << std::endl;
            return true; 
        }

        // 🕒 [3. 슬라이딩 윈도우 업데이트]
        vote_window_.push_front(motion_id);
        if ((int)vote_window_.size() > window_size_) vote_window_.pop_back();

        int final_motion_id = motion_id; // 기본값은 최신 명령

        // 🗳️ [4. Active Zone Voting] 마지막 전 2개 구간에서만 투표 결과 적용
        if (current_seg_idx_ >= last_idx - 2) {
            std::map<int, int> counts;
            for (int id : vote_window_) counts[id]++;
            
            int winner = -1, max_votes = -1;
            for (auto const& [id, count] : counts) {
                if (count > max_votes) { max_votes = count; winner = id; }
            }
            final_motion_id = winner;
            std::cout << "[SDK] 투표 구간 진입: 최빈값 " << final_motion_id << " (" << max_votes << "표) 선택" << std::endl;
        }

        if (pending_motion_id_ == final_motion_id) return true;
        
        // --- 궤적 재생성 로직 ---
        const auto& target_seq = motion_library_.at(final_motion_id);
        auto& last_seg = active_trajectory_.back();
        std::vector<double> q_last(NUMBER_OF_JOINTS);
        double T_seg = static_cast<double>(last_seg.total_ticks) / HZ;
        
        for(int i=0; i<NUMBER_OF_JOINTS; ++i) {
            double dq_seg = (last_seg.c3[i]*pow(T_seg,3) + last_seg.c4[i]*pow(T_seg,4) + last_seg.c5[i]*pow(T_seg,5)) 
                          + last_seg.c1[i]*T_seg + last_seg.c2[i]*T_seg*T_seg;
            q_last[i] = last_seg.c0[i] + dq_seg;
        }
        
        pending_trajectory_.clear();
        std::vector<std::vector<double>> waypoints = {q_last};
        for (const auto& p : target_seq.poses) waypoints.push_back(p);
        
        std::vector<double> durations = {transition_time};
        for (double d : target_seq.durations) durations.push_back(d);

        int M = waypoints.size();
        std::vector<std::vector<double>> V(M, std::vector<double>(NUMBER_OF_JOINTS, 0.0));
        
        for (int i = 0; i < M; ++i) {
            if (i == 0) {
                for (int j = 0; j < NUMBER_OF_JOINTS; ++j) 
                    V[i][j] = normalize_angle(waypoints[1][j] - last_seg.c0[j]) / (T_seg + durations[0]);
                
                if (current_seg_idx_ < last_idx) {
                    std::vector<double> v0_last(NUMBER_OF_JOINTS), a0_last(NUMBER_OF_JOINTS);
                    for(int j=0; j<NUMBER_OF_JOINTS; ++j) { v0_last[j] = last_seg.c1[j]; a0_last[j] = 2.0 * last_seg.c2[j]; }
                    calculate_coefficients(last_seg.c0, q_last, v0_last, V[i], a0_last, std::vector<double>(NUMBER_OF_JOINTS, 0.0), T_seg, last_seg);
                }
                for (int j = 0; j < NUMBER_OF_JOINTS; ++j) {
                    double T_l = static_cast<double>(last_seg.total_ticks) / HZ;
                    V[i][j] = last_seg.c1[j] + 2*last_seg.c2[j]*T_l + 3*last_seg.c3[j]*pow(T_l,2) + 4*last_seg.c4[j]*pow(T_l,3) + 5*last_seg.c5[j]*pow(T_l,4);
                }
            } else if (i == M - 1) {
                V[i].assign(NUMBER_OF_JOINTS, 0.0);
            } else {
                for (int j = 0; j < NUMBER_OF_JOINTS; ++j) 
                    V[i][j] = normalize_angle(waypoints[i+1][j] - waypoints[i-1][j]) / (durations[i-1] + durations[i]);
            }
        }

        for (int i = 0; i < M - 1; ++i) {
            TrajectorySegment seg;
            calculate_coefficients(waypoints[i], waypoints[i+1], V[i], V[i+1], std::vector<double>(NUMBER_OF_JOINTS, 0.0), std::vector<double>(NUMBER_OF_JOINTS, 0.0), durations[i], seg);
            pending_trajectory_.push_back(seg);
        }
        pending_motion_id_ = final_motion_id;
        std::cout << "[SDK] 예약 확정: 모션 ID " << final_motion_id << std::endl;
        return true;
    }

    // --- 정지 상태 로직 ---
    active_trajectory_.clear(); pending_trajectory_.clear(); pending_motion_id_ = -1;
    vote_window_.clear(); 
    current_motion_id_ = motion_id;
    const auto& seq = motion_library_.at(motion_id);
    std::vector<std::vector<double>> waypoints = {std::vector<double>(current_pose, current_pose + NUMBER_OF_JOINTS)};
    for (const auto& p : seq.poses) waypoints.push_back(p);
    
    std::vector<double> durations = {transition_time};
    for (double d : seq.durations) durations.push_back(d);

    int M = waypoints.size();
    std::vector<std::vector<double>> V(M, std::vector<double>(NUMBER_OF_JOINTS, 0.0));
    for (int i = 1; i < M - 1; ++i) {
        for (int j = 0; j < NUMBER_OF_JOINTS; ++j) 
            V[i][j] = normalize_angle(waypoints[i+1][j] - waypoints[i-1][j]) / (durations[i-1] + durations[i]);
    }

    for (int i = 0; i < M - 1; ++i) {
        TrajectorySegment seg;
        calculate_coefficients(waypoints[i], waypoints[i+1], V[i], V[i+1], std::vector<double>(NUMBER_OF_JOINTS, 0.0), std::vector<double>(NUMBER_OF_JOINTS, 0.0), durations[i], seg);
        active_trajectory_.push_back(seg);
    }
    current_seg_idx_ = 0; current_tick_ = 0;
    return true;
}

bool SDK_Motion::Is_Moving() { return !active_trajectory_.empty() && (current_seg_idx_ < (int)active_trajectory_.size()); }

bool SDK_Motion::Get_Next_Tick(double* All_Theta) {
    if (!Is_Moving()) {
        if (!pending_trajectory_.empty()) {
            active_trajectory_ = std::move(pending_trajectory_);
            pending_trajectory_.clear();
            vote_window_.clear(); // 🧹 다음 투표 준비
            current_motion_id_ = pending_motion_id_; pending_motion_id_ = -1;
            current_seg_idx_ = 0; current_tick_ = 0;
        } else {
            current_v_.assign(NUMBER_OF_JOINTS, 0.0); current_a_.assign(NUMBER_OF_JOINTS, 0.0);
            return false; 
        }
    }
    const auto& seg = active_trajectory_[current_seg_idx_];
    double t = static_cast<double>(current_tick_) / HZ;
    for (int i = 0; i < NUMBER_OF_JOINTS; ++i) {
        double t2 = t*t, t3 = t2*t, t4 = t3*t, t5 = t4*t;
        All_Theta[i] = seg.c0[i] + seg.c1[i]*t + seg.c2[i]*t*t + seg.c3[i]*t*t*t + seg.c4[i]*t*t*t*t + seg.c5[i]*t*t*t*t*t;
        current_v_[i] = seg.c1[i] + 2*seg.c2[i]*t + 3*seg.c3[i]*t*t + 4*seg.c4[i]*t*t*t + 5*seg.c5[i]*t*t*t*t;
        current_a_[i] = 2*seg.c2[i] + 6*seg.c3[i]*t + 12*seg.c4[i]*t*t + 20*seg.c5[i]*t*t*t;
    }
    current_tick_++;
    if (current_tick_ >= seg.total_ticks) { current_seg_idx_++; current_tick_ = 0; }
    return true; 
}

""" 1. 슬라이딩 윈도우(Sliding Window) 투표: std::deque를 사용하여 최근 20개의 명령만 유지합니다. 이를 통해 오래된 데이터의 영향력을 자동으로 배제하고 최신 명령의 트렌드를 즉각
      반영합니다.
     2. 80% Lock-out (진행률 기반 잠금): 현재 재생 중인 구간(Segment)이 80% 이상 진행되면 예약을 잠금(Lock) 처리합니다. 이는 다음 구간으로 넘어가기 직전에 궤적을 급하게 수정할 경우
      발생할 수 있는 물리적 충격(Jump)을 방지합니다.
   3. 집중 투표 구간(Active Zone Voting):
       * 초반 구간: 가장 최신 명령을 즉시 예약에 반영하여 빠른 반응성을 유지합니다.
       * 마지막 전 2개 포즈 구간: 이 시점부터는 슬라이딩 윈도우 내의 최빈값을 계산하여 신중하게 최종 예약을 확정합니다.
       * 마지막 구간: 기존 로직과 동일하게 모든 예약을 거부하여 안전하게 동작을 마무리합니다.
   4. 계산 최적화: 최빈값 모션 ID가 현재 예약된 ID와 동일할 경우에는 무거운 궤적 재계산 과정을 생략하도록 효율화했습니다.

  이제 로봇은 움직이는 동안 명령을 계속 수집하다가, 동작이 끝나기 직전 가장 지배적인 명령을 최종 선택하여 부드럽게 다음 동작으로 연결합니다. 터미널의 [SDK] 투표 구간 진입, [SDK] 예약
  잠금 등의 로그를 통해 실시간 판단 상태를 확인하실 수 있습니다. """
                                                                         