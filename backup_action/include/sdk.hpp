#pragma once
#include <vector>
#include <map>

#define NUMBER_OF_JOINTS 22
#define HZ 100.0

// 🎯 궤적 구간 구조체 (5차 다항식 계수들)
struct TrajectorySegment {
    std::vector<double> c0, c1, c2, c3, c4, c5;
    int total_ticks;
};

// 🎯 모션 구조체 (포즈 리스트와 시간)
struct MotionSequence {
    std::vector<std::vector<double>> poses;
    std::vector<double> durations;
};

class SDK_Motion 
{
private:
    std::map<int, MotionSequence> motion_library_;
    std::vector<TrajectorySegment> planned_trajectory_;
    
    int current_seg_idx_;
    int current_tick_;

    void define_motions();
    void calculate_coefficients(
        const std::vector<double>& q0, const std::vector<double>& qf,
        const std::vector<double>& v0, const std::vector<double>& vf,
        const std::vector<double>& a0, const std::vector<double>& af,
        double T, TrajectorySegment& segment);

public:
    SDK_Motion();

    // 부팅 시 기본 자세를 세팅해주는 함수
    void Init_Default_Pose(double* All_Theta);

    // 명령을 받으면 전체 궤적을 굽는 함수
    bool Generate_Trajectory(int motion_id, double* current_pose);

    // 100Hz로 불리면서 다음 0.01초의 각도를 뱉어내는 함수
    bool Get_Next_Tick(double* All_Theta);
};