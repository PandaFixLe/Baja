#include "mock_path_generator.h"
#include <algorithm>
#include <iostream>
#include <cmath>

MockPathGenerator::MockPathGenerator() 
    : current_idx_(0), target_idx_(0), current_x_(0.0), current_y_(0.0), current_heading_(0.0) {
    std::cout << "[MockPathGenerator] 初始化完成\n";
}

MockPathGenerator::~MockPathGenerator() {}

void MockPathGenerator::Clear() {
    path_.clear();
    current_idx_ = 0;
    target_idx_ = 0;
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_heading_ = 0.0;
}

void MockPathGenerator::Reset() {
    current_idx_ = 0;
    target_idx_ = 0;
}

bool MockPathGenerator::IsFinished() const {
    return current_idx_ >= path_.size();
}

PathPoint MockPathGenerator::GetNextPoint() {
    if (current_idx_ < path_.size()) {
        return path_[current_idx_++];
    }
    // 返回最后一个点
    if (!path_.empty()) {
        return path_.back();
    }
    return PathPoint();
}

size_t MockPathGenerator::GetClosestIndex(double car_x, double car_y) {
    if (path_.empty()) return 0;

    size_t search_start = (target_idx_ > 12) ? (target_idx_ - 12) : 0;
    size_t search_end = std::min(path_.size(), target_idx_ + 30);
    size_t closest_idx = search_start;
    double min_dist = 1e9;

    for (size_t i = search_start; i < search_end; ++i) {
        double d = std::hypot(path_[i].x - car_x, path_[i].y - car_y);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    return closest_idx;
}

PathPoint MockPathGenerator::GetClosestPoint(double car_x, double car_y) {
    if (path_.empty()) return PathPoint();
    return path_[GetClosestIndex(car_x, car_y)];
}

// 在 mock_path_generator.cpp 中实现
PathPoint MockPathGenerator::GetTargetPoint(
    double car_x,
    double car_y,
    double car_heading,
    double lookahead_dist) {
    if (path_.empty()) return PathPoint();

    // 1. 仅在当前目标点附近向前搜索最近点，避免目标点在 S 弯中来回跳变
    size_t search_start = (target_idx_ > 8) ? (target_idx_ - 8) : 0;
    size_t search_end = std::min(path_.size(), target_idx_ + 25);
    size_t closest_idx = search_start;
    double min_dist = 1e9;
    for (size_t i = search_start; i < search_end; ++i) {
        double d = std::hypot(path_[i].x - car_x, path_[i].y - car_y);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // 2. 仅接受车前方的点，避免在弯道/回头弯里选中车后方目标点
    const double heading_x = std::cos(car_heading);
    const double heading_y = std::sin(car_heading);
    const double min_forward_projection = 0.3;
    const size_t max_forward_steps = 20;

    // 3. 沿路径弧长向前查找并插值，减少离散点跳变
    double accumulated_dist = 0.0;
    PathPoint best_forward_point = path_[closest_idx];
    bool found_forward_point = false;

    for (size_t i = closest_idx; i < path_.size() - 1; ++i) {
        if (i - closest_idx > max_forward_steps) {
            break;
        }

        const PathPoint& seg_start = path_[i];
        const PathPoint& seg_end = path_[i + 1];
        double seg_len = std::hypot(seg_end.x - seg_start.x, seg_end.y - seg_start.y);
        if (seg_len < 1e-6) continue;

        const PathPoint& candidate = seg_end;
        double rel_x = candidate.x - car_x;
        double rel_y = candidate.y - car_y;
        double forward_projection = rel_x * heading_x + rel_y * heading_y;

        if (forward_projection > min_forward_projection) {
            best_forward_point = candidate;
            found_forward_point = true;
        }

        if (accumulated_dist + seg_len >= lookahead_dist && forward_projection > min_forward_projection) {
            double remain = lookahead_dist - accumulated_dist;
            double ratio = std::clamp(remain / seg_len, 0.0, 1.0);

            PathPoint interpolated;
            interpolated.x = seg_start.x + ratio * (seg_end.x - seg_start.x);
            interpolated.y = seg_start.y + ratio * (seg_end.y - seg_start.y);
            interpolated.heading = seg_start.heading + ratio * (seg_end.heading - seg_start.heading);
            interpolated.curvature = seg_start.curvature + ratio * (seg_end.curvature - seg_start.curvature);
            interpolated.speed_limit = seg_start.speed_limit + ratio * (seg_end.speed_limit - seg_start.speed_limit);
            interpolated.timestamp = seg_start.timestamp + ratio * (seg_end.timestamp - seg_start.timestamp);
            target_idx_ = i;
            return interpolated;
        }

        accumulated_dist += seg_len;
    }

    if (found_forward_point) {
        target_idx_ = closest_idx;
        return best_forward_point;
    }

    target_idx_ = path_.size() - 1;
    return path_.back();
}

void MockPathGenerator::AddPoint(double x, double y, double heading, double curvature, double speed_limit) {
    double timestamp = path_.empty() ? 0.0 : path_.back().timestamp + dt_;
    path_.emplace_back(x, y, heading, curvature, speed_limit, timestamp);
}

double MockPathGenerator::CalculateHeading(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

double MockPathGenerator::CalculateCurvature(double radius) {
    if (std::abs(radius) < 1e-6) {
        return 0.0;
    }
    return 1.0 / radius;
}

void MockPathGenerator::GenerateStraightLine(double length) {
    path_.clear();
    current_idx_ = 0;
    
    double num_points = length / 0.5;  // 每 0.5m 一个点
    for (int i = 0; i <= num_points; ++i) {
        PathPoint pt;
        pt.x = i * 0.5;
        pt.y = 0.0;
        pt.heading = 0.0;
        pt.curvature = 0.0;
        pt.speed_limit = 5.0;
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
        pt.curvature = 1.0 / radius;
        pt.speed_limit = 2.0;
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成圆形路径，半径=" << radius << "m, 点数=" << path_.size() << "\n";
}

void MockPathGenerator::GenerateSineWave(double amplitude, double wavelength) {
    path_.clear();
    current_idx_ = 0;
    
    double total_length = wavelength * 2;  // 两个周期
    int num_points = static_cast<int>(total_length / 0.5);
    
    for (int i = 0; i < num_points; ++i) {
        PathPoint pt;
        double x = i * 0.5;
        double y = amplitude * std::sin(2 * M_PI * x / wavelength);
        
        pt.x = x;
        pt.y = y;
        pt.heading = std::atan2(amplitude * 2 * M_PI / wavelength * std::cos(2 * M_PI * x / wavelength), 1.0);
        pt.curvature = 0.05;  // 简化曲率
        pt.speed_limit = 3.0;
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成S弯路径，振幅=" << amplitude << "m, 波长=" << wavelength << "m\n";
}

void MockPathGenerator::GenerateSineWaveAppend(double amplitude, double wavelength) {
    double total_length = wavelength * 2;  // 两个周期
    int num_points = static_cast<int>(total_length / 0.5);
    
    for (int i = 0; i < num_points; ++i) {
        double s = i * 0.5;  // 沿路径的距离
        
        // 计算横向偏移
        double offset = amplitude * std::sin(2 * M_PI * s / wavelength);
        
        // 前进方向的单位向量
        double dir_x = std::cos(current_heading_);
        double dir_y = std::sin(current_heading_);
        
        // 垂直方向的单位向量（向左）
        double perp_x = -dir_y;
        double perp_y = dir_x;
        
        // 位置计算
        double x = current_x_ + s * dir_x + offset * perp_x;
        double y = current_y_ + s * dir_y + offset * perp_y;
        
        // 计算航向角（切线方向）
        double dy_ds = amplitude * 2 * M_PI / wavelength * std::cos(2 * M_PI * s / wavelength);
        double heading = current_heading_ + std::atan2(dy_ds, 1.0);
        
        // 计算曲率
        double d2y_ds2 = -amplitude * std::pow(2 * M_PI / wavelength, 2) * 
                         std::sin(2 * M_PI * s / wavelength);
        double curvature = d2y_ds2 / std::pow(1 + dy_ds * dy_ds, 1.5);
        
        AddPoint(x, y, heading, curvature, 3.0);
    }
    
    // 更新终点位置（S弯结束时offset回到0，所以只前进total_length）
    current_x_ += total_length * std::cos(current_heading_);
    current_y_ += total_length * std::sin(current_heading_);
}
void MockPathGenerator::GenerateStraight(double length, double speed_limit) {
    double step = 0.5;
    int num_points = static_cast<int>(length / step);
    
    for (int i = 0; i <= num_points; ++i) {
        double x = current_x_ + i * step * std::cos(current_heading_);
        double y = current_y_ + i * step * std::sin(current_heading_);
        AddPoint(x, y, current_heading_, 0.0, speed_limit);
    }
    
    current_x_ += length * std::cos(current_heading_);
    current_y_ += length * std::sin(current_heading_);
}

void MockPathGenerator::GenerateCurve(double radius, double angle_deg, bool clockwise, double speed_limit) {
    double angle_rad = angle_deg * M_PI / 180.0;
    int num_points = static_cast<int>(std::abs(angle_deg) / 2.0);
    double angle_step = angle_rad / num_points;
    
    double direction = clockwise ? -1.0 : 1.0;
    double center_x = current_x_ - direction * radius * std::sin(current_heading_);
    double center_y = current_y_ + direction * radius * std::cos(current_heading_);
    
    double start_angle = std::atan2(current_y_ - center_y, current_x_ - center_x);
    
    for (int i = 0; i <= num_points; ++i) {
        double current_angle = start_angle + direction * i * angle_step;
        double x = center_x + radius * std::cos(current_angle);
        double y = center_y + radius * std::sin(current_angle);
        double heading = current_angle + (clockwise ? -M_PI/2 : M_PI/2);
        
        AddPoint(x, y, heading, direction / radius, speed_limit);
    }
    
    current_x_ = center_x + radius * std::cos(start_angle + direction * angle_rad);
    current_y_ = center_y + radius * std::sin(start_angle + direction * angle_rad);
    current_heading_ += direction * angle_rad;
}

void MockPathGenerator::GenerateCompetitionTrack() {
    Clear();
    std::cout << "=== 生成比赛赛道 ===\n";
    
    GenerateStraight(20.0, 4.0);
    GenerateSineWaveAppend(12.0, 25.0);
    GenerateCurve(15.0, 90.0, true, 3.5);
    GenerateStraight(15.0, 4.0);
    GenerateCurve(8.0, 180.0, true, 1.5);
    GenerateStraight(30.0, 6.0);
    
    std::cout << "赛道生成完成！总点数：" << path_.size() << "\n";
}

void MockPathGenerator::GenerateHairpinTestSection() {
    Clear();
    std::cout << "=== 生成发卡弯测试段 ===\n";
    
    GenerateStraight(15.0, 2.0);
    GenerateCurve(6.0, 180.0, true, 1.2);
    GenerateStraight(5.0, 2.0);
    GenerateCurve(10.0, 180.0, false, 1.8);
    
    std::cout << "发卡弯测试段完成！\n";
}

void MockPathGenerator::GenerateHighSpeedSection() {
    Clear();
    std::cout << "=== 生成高速循迹测试段 ===\n";
    
    GenerateStraight(20.0, 5.0);
    GenerateCurve(30.0, 45.0, true, 6.0);
    GenerateStraight(30.0, 7.0);
    
    std::cout << "高速循迹测试段完成！\n";
}

void MockPathGenerator::GenerateOffroadObstacles() {
    Clear();
    std::cout << "=== 生成越野障碍段 ===\n";
    
    GenerateStraight(10.0, 3.0);
    
    // 模拟搓板路
    for (int i = 0; i < 50; ++i) {
        double x = current_x_ + i * 0.2 * std::cos(current_heading_);
        double y = current_y_ + i * 0.2 * std::sin(current_heading_);
        AddPoint(x, y, current_heading_, 0.0, 2.0);
    }
    current_x_ += 10.0 * std::cos(current_heading_);
    current_y_ += 10.0 * std::sin(current_heading_);
    
    GenerateSineWave(8.0, 15.0);
    
    std::cout << "越野障碍段完成！\n";
}
