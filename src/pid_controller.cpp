#include "pid_controller.h"
#include <iostream>

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), prev_error_(0) {}

double PIDController::ComputeSteering(double error, double speed, double curvature, double current_heading) {
    // 简化版 PID（实际需加积分限幅、微分滤波）
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;
    
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    std::cout << "[PID] output=" << output << "\n";
    return output;
}

std::string PIDController::GetName() const {
    return "PID_Controller";
}

void PIDController::SetGains(double p, double i, double d) {
    kp_ = p; ki_ = i; kd_ = d;
    std::cout << "[PID] 参数更新: " << kp_ << "/" << ki_ << "/" << kd_ << "\n";
}