#include "mock_path_generator.h"
#include <iostream>

MockPathGenerator::MockPathGenerator() {
    std::cout << "[MockPathGenerator] 初始化完成\n";
}

void MockPathGenerator::GenerateStraightLine(double length) {
    path_.clear();
    current_idx_ = 0;
    
    double num_points = length / 0.5;  // 每 0.5m 一个点
    for (int i = 0; i <= num_points; ++i) {
        PathPoint pt;
        pt.x = i * 0.5;
        pt.y = 0.0;
        pt.heading = 0.0;  // 直线，航向角为 0
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成直线路径，长度=" << length << "m, 点数=" << path_.size() << "\n";
}

void MockPathGenerator::GenerateCircle(double radius) {
    path_.clear();
    current_idx_ = 0;
    
    int num_points = 100;  // 100 个点画圆
    double angle_step = 2 * M_PI / num_points;
    
    for (int i = 0; i < num_points; ++i) {
        PathPoint pt;
        double angle = i * angle_step;
        pt.x = radius * std::cos(angle);
        pt.y = radius * std::sin(angle);
        pt.heading = angle + M_PI / 2;  // 切线方向
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成圆形路径，半径=" << radius << "m, 点数=" << path_.size() << "\n";
}

void MockPathGenerator::GenerateSineWave(double amplitude, double wavelength) {
    path_.clear();
    current_idx_ = 0;
    
    double total_length = wavelength * 2;  // 2 个周期
    int num_points = total_length / 0.5;
    
    for (int i = 0; i <= num_points; ++i) {
        PathPoint pt;
        double x = i * 0.5;
        double y = amplitude * std::sin(2 * M_PI * x / wavelength);
        
        pt.x = x;
        pt.y = y;
        // 计算切线方向（导数）
        pt.heading = std::atan2(amplitude * 2 * M_PI / wavelength * std::cos(2 * M_PI * x / wavelength), 1.0);
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成正弦波路径，振幅=" << amplitude << "m, 波长=" << wavelength << "m\n";
}

PathPoint MockPathGenerator::GetNextPoint() {
    if (current_idx_ >= path_.size()) {
        // 循环播放
        current_idx_ = 0;
    }
    return path_[current_idx_++];
}

bool MockPathGenerator::IsFinished() const {
    return current_idx_ >= path_.size();
}