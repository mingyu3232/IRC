#pragma once
#include <vector>
#include <map>
#include <deque>

#define NUMBER_OF_JOINTS 22
#define HZ 100.0

// 궤적 구간 구조체 (5차 다항식 계수들)
struct TrajectorySegment {
    std::vector<double> c0, c1, c2, c3, c4, c5; 
    int total_ticks; 
};

// 모션 구조체 (포즈 리스트와 시간)
struct MotionSequence {
    std::vector<std::vector<double>> poses; 
    std::vector<double> durations; 
};

class SDK_Motion 
{
private:
    std::map<int, MotionSequence> motion_library_; 

    std::vector<TrajectorySegment> active_trajectory_; 
    std::vector<TrajectorySegment> pending_trajectory_; 

    int current_seg_idx_; 
    int current_tick_; 
    int current_motion_id_; 
    int pending_motion_id_; 
    
    std::deque<int> vote_window_; // 🕒 슬라이딩 윈도우: 최근 N개의 명령 저장
    const int window_size_ = 20;   // 윈도우 크기

    std::vector<double> current_v_; 
    std::vector<double> current_a_; 

    void define_motions(); 
    void calculate_coefficients( 
        const std::vector<double>& q0, const std::vector<double>& qf,
        const std::vector<double>& v0, const std::vector<double>& vf,
        const std::vector<double>& a0, const std::vector<double>& af,
        double T, TrajectorySegment& segment); 

public:
    SDK_Motion();
    void Init_Default_Pose(double* All_Theta);
    bool Generate_Trajectory(int motion_id, double* current_pose, double transition_time = 0.5);
    bool Get_Next_Tick(double* All_Theta);
    bool Is_Moving();
};