#include "pure_pursuit_controller.h"
#include <iostream>
#include <cmath>   // for std::atan2, std::tan, std::clamp
#include <algorithm> // std::clamp 在这个头文件里

PurePursuitController::PurePursuitController(double lookahead, double wheelbase, double max_steer)
    : lookahead_(lookahead),
      wheelbase_(wheelbase),
      max_steer_angle_(max_steer),
      k_gain_(0.3),           // 默认值，可通过SetDynamicParameters修改
      L_min_(1.5),            // 默认值
      L_max_(4.0),            // 默认值
      curvature_threshold_(0.15),  // 默认曲率阈值（对应半径约6.7m的发卡弯）
      emergency_lookahead_(0.8),   // 默认急弯预瞄距离
      emergency_mode_active_(false),
      emergency_log_printed_(false)  // ✅ 新增：初始化为 false
{
    // 工程安全：参数有效性检查
    if (lookahead_ <= 0) {
        std::cerr << "[PurePursuit] 警告：预瞄距离必须 > 0，已修正为 1.0" << std::endl;
        lookahead_ = 1.0;
    }
    
    if (wheelbase_ <= 0) {
        std::cerr << "[PurePursuit] 警告：轴距必须 > 0，已修正为 1.5" << std::endl;
        wheelbase_ = 1.5;
    }
    
    if (max_steer_angle_ <= 0 || max_steer_angle_ > 1.0) {
        std::cerr << "[PurePursuit] 警告：最大转向角应在 (0, 1.0] rad范围，已修正为 0.6" << std::endl;
        max_steer_angle_ = 0.6;
    }
}

double PurePursuitController::ComputeSteering(double error, double speed, double curvature, double current_heading) {
    (void)current_heading;
    // 1. 根据速度和曲率动态调整预瞄距离 (这个保留，用于调参)
    UpdateParameters(speed, curvature);
    
    double dynamic_lookahead = lookahead_;
    
    // 2. ✅ 修正核心算法：Pure Pursuit 必须基于几何误差计算
    // 公式：k = 2 * lateral_error / L^2
    // 注意：error 是有符号的，决定了向左还是向右转
    
    // 删除之前的 if (curvature != 0.0) 判断！
    // 强制使用反馈控制公式
    double path_curvature = (2.0 * error) / (dynamic_lookahead * dynamic_lookahead);
    
    // 3. 计算转向角：delta = atan(L * k)
    double steering_angle = std::atan2(wheelbase_ * path_curvature, 1.0);
    
    // 4. 转向限幅
    steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);
    
    // 调试输出（可选）
    // std::cout << "[PP] error=" << error << " L=" << dynamic_lookahead 
    //           << " -> steer=" << steering_angle << "\n";
    
    return steering_angle;
}

std::string PurePursuitController::GetName() const {
    return "PurePursuit_Controller";
}

void PurePursuitController::SetLookahead(double lookahead) {
    if (lookahead > 0.5) {  // 最小预瞄距离保护
        lookahead_ = lookahead;
        std::cout << "[PurePursuit] 预瞄距离更新：" << lookahead_ << " m\n";
    } else {
        std::cerr << "[PurePursuit] 警告：预瞄距离不能 < 0.5m\n";
    }
}

// 新增：根据速度和曲率动态更新参数
void PurePursuitController::UpdateParameters(double speed, double curvature) {
    // 1. 计算目标预瞄距离（保持你原有的逻辑）
    double base_lookahead = L_min_ + k_gain_ * speed;
    double abs_curvature = std::abs(curvature);
    
    if (abs_curvature > curvature_threshold_) {
        double curvature_factor = curvature_threshold_ / abs_curvature;
        base_lookahead = std::max(emergency_lookahead_, base_lookahead * curvature_factor);
    }
    base_lookahead = std::clamp(base_lookahead, L_min_, L_max_);

    // 🔹 新增：判断是否处于“紧急模式”（超急弯）
    bool is_emergency = (abs_curvature > 0.25);

    if (is_emergency) {
        // 超急弯：强制缩短预瞄距离，并标记为紧急状态
        lookahead_ = std::max(0.6, base_lookahead * 0.5);
        emergency_mode_active_ = true;
        
        // ✅ 修改：只在变化较大时打印日志，避免重复
        if (std::abs(lookahead_ - base_lookahead) > 0.3 && !emergency_log_printed_) {
            std::cout << "[PurePursuit] 🚨 超急弯！预瞄紧急缩短至" << lookahead_ << "m (曲率=" << abs_curvature << ")\n";
            emergency_log_printed_ = true;  // 标记已打印
        }
    } else {
        // 恢复阶段：缓慢增加预瞄距离，避免突变
        if (emergency_mode_active_) {
            // 渐进恢复：每次只增加 10%
            double recovery_factor = 0.1;
            double target_lookahead = lookahead_ * (1 - recovery_factor) + base_lookahead * recovery_factor;
            lookahead_ = std::min(target_lookahead, base_lookahead); // 不超过正常值
        } else {
            // 正常情况：使用低通滤波（20%权重）
            double smoothing_factor = 0.2;
            double target_lookahead = lookahead_ * (1 - smoothing_factor) + base_lookahead * smoothing_factor;
            lookahead_ = target_lookahead;
    
        }

        // 只有变化较大时才打印日志
        if (std::abs(lookahead_ - base_lookahead) > 0.3) {
            std::cout << "[PurePursuit] ⚠️ 预瞄大幅调整: " << base_lookahead << "m -> " << lookahead_ << "m\n";
        }

        emergency_mode_active_ = false;
        emergency_log_printed_ = false;  // 重置标志位
    }
}

// 新增：设置动态预瞄参数
void PurePursuitController::SetDynamicParameters(double k_gain, double L_min, double L_max) {
    if (k_gain > 0 && L_min > 0 && L_max > L_min) {
        k_gain_ = k_gain;
        L_min_ = L_min;
        L_max_ = L_max;
        std::cout << "[PurePursuit] 动态参数更新: k_gain=" << k_gain_ 
                  << ", L_min=" << L_min_ << ", L_max=" << L_max_ << std::endl;
    } else {
        std::cerr << "[PurePursuit] 错误：动态参数无效！" << std::endl;
    }
}

// 新增：设置曲率相关参数
void PurePursuitController::SetCurvatureParameters(double threshold, double emergency_lookahead) {
    if (threshold > 0 && emergency_lookahead > 0.5) {
        curvature_threshold_ = threshold;
        emergency_lookahead_ = emergency_lookahead;
        std::cout << "[PurePursuit] 曲率参数更新: threshold=" << curvature_threshold_ 
                  << ", emergency_lookahead=" << emergency_lookahead_ << std::endl;
    } else {
        std::cerr << "[PurePursuit] 错误：曲率参数无效！" << std::endl;
    }
}