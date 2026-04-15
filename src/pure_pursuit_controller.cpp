#include "pure_pursuit_controller.h"
#include <iostream>
#include <cmath>  // for std::atan2, std::tan, std::clamp
#include <algorithm>  // std::clamp 在这个头文件里

PurePursuitController::PurePursuitController(double lookahead, double wheelbase, double max_steer)
    : lookahead_(lookahead), 
      wheelbase_(wheelbase), 
      max_steer_angle_(max_steer) 
{
    // 工程安全：参数有效性检查
    if (lookahead_ <= 0) {
        std::cerr << "[PurePursuit] 警告: 预瞄距离必须 > 0，已修正为 1.0" << std::endl;
        lookahead_ = 1.0;
    }
    if (wheelbase_ <= 0) {
        std::cerr << "[PurePursuit] 警告: 轴距必须 > 0，已修正为 1.5" << std::endl;
        wheelbase_ = 1.5;
    }
}
// 1232131
double PurePursuitController::ComputeSteering(double error, double speed) {
    // 🎯 动态预瞄核心参数（实车经验初值）
    double k_gain = 0.3;      // 速度增益系数：车速每+1m/s，预瞄+0.5m
    double L_min = 1.5;       // 最小预瞄：防低速震荡
    double L_max = 4.0;       // 最大预瞄：防高速反应迟钝

    // 计算动态预瞄距离
    double dynamic_lookahead = std::clamp(
        L_min + k_gain * speed,
        L_min, L_max
    );

    // 纯追踪几何公式
    double curvature = (2.0 * error) / (dynamic_lookahead * dynamic_lookahead);
    double steering_angle = std::atan2(wheelbase_ * curvature, 1.0);
    
    // 转向限幅
    steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);

    return steering_angle;
}
std::string PurePursuitController::GetName() const {
    return "PurePursuit_Controller";
}

void PurePursuitController::SetLookahead(double lookahead) {
    if (lookahead > 0.5) {  // 最小预瞄距离保护
        lookahead_ = lookahead;
        std::cout << "[PurePursuit] 预瞄距离更新: " << lookahead_ << " m\n";
    } else {
        std::cerr << "[PurePursuit] 警告: 预瞄距离不能 < 0.5m\n";
    }
}