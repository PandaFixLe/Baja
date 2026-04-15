#include "stanley_controller.h"
#include <cmath>
#include <algorithm>

StanleyController::StanleyController(double kp, double ktheta, double max_steer)
    : k_p_(kp), k_theta_(ktheta), max_steer_(max_steer) {}

double StanleyController::ComputeSteering(double lateral_error, double speed) {
    // 🛡️ 防零速除错
    double v_safe = std::max(speed, 0.5); 
    
    // Stanley 核心公式：δ = θ_e + atan2(k*e / v, 1)
    // 此处简化假设航向误差 θ_e ≈ 0（实际需从感知获取路径切线角）
    double theta_e = 0.0; 
    
    double delta = theta_e + std::atan2(k_p_ * lateral_error / v_safe, 1.0);
    return std::clamp(delta, -max_steer_, max_steer_);
}