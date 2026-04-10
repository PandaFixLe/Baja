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

double PurePursuitController::ComputeSteering(double error, double speed) {
    // 🛡️ 输入有效性检查（工程红线）
    if (lookahead_ < 0.1) {
        std::cerr << "[PurePursuit] 错误: 预瞄距离过小!" << std::endl;
        return 0.0;
    }
    
    // 📐 纯追踪核心几何公式
    // 简化版：假设 error 是横向偏差，预瞄点在正前方
    // 曲率 κ = 2 * sin(α) / Ld ≈ 2 * error / Ld² (小角度近似)
    double curvature = (2.0 * error) / (lookahead_ * lookahead_);
    
    // 🚗 自行车模型：曲率 → 前轮转角
    // δ = atan2(L * κ, 1)
    double steering_angle = std::atan2(wheelbase_ * curvature, 1.0);
    
    // 🚨 工程安全：转向角限幅（防止机械损坏/翻车）
    steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);
    
    // 📊 调试输出（实际项目中可用日志库替代）
    std::cout << "[PurePursuit] 误差=" << error 
              << " 预瞄=" << lookahead_ 
              << " 曲率=" << curvature 
              << " -> 转角=" << steering_angle << " rad\n";
    
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