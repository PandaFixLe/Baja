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

// pure_pursuit_controller.cpp - 修改实现
double PurePursuitController::ComputeSteering(
    double lateral_error_raw, 
    double speed, 
    double path_curvature, 
    double current_heading) {
    (void)current_heading;

    // 1. 动态调整预瞄距离
    UpdateParameters(speed, path_curvature);
    const double min_safe_lookahead = std::max(L_min_, wheelbase_ * 1.5);
    double dynamic_lookahead = std::clamp(lookahead_, min_safe_lookahead, L_max_);
    const double local_curv = std::abs(path_curvature);
    const double max_lookahead_for_curve =
        (local_curv < 0.05) ? L_max_ : ((local_curv < 0.15) ? 3.0 : 2.0);
    dynamic_lookahead = std::min(dynamic_lookahead, max_lookahead_for_curve);

    // 2. Pure Pursuit 几何公式
    double required_curvature =
        (2.0 * lateral_error_raw) / (dynamic_lookahead * dynamic_lookahead);

    // 3. 限制目标曲率，避免预瞄过短或误差过大时直接打满方向
    const double max_curvature = std::tan(max_steer_angle_) / wheelbase_;
    required_curvature = std::clamp(required_curvature, -max_curvature, max_curvature);

    // 4. 反馈主导，前馈只在方向一致或误差较小时轻量叠加，避免“打架”
    double feedback = std::atan(wheelbase_ * required_curvature);
    double feedforward = std::atan(wheelbase_ * path_curvature);
    const bool same_direction = (feedback * feedforward >= 0.0);
    double feedforward_weight = 0.20;
    if (!same_direction) {
        feedforward_weight = 0.0;
    }
    if (local_curv > 0.05 && local_curv < 0.2 && same_direction) {
        feedforward_weight = 0.40;
    }
    if (std::abs(lateral_error_raw) < 0.2 && same_direction) {
        feedforward_weight = std::max(feedforward_weight, 0.30);
    }

    double steering_angle = feedforward * feedforward_weight + feedback * (1.0 - feedforward_weight);
    
    // 5. 绝对值限幅（先限幅，再做变化率限制）
    steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);

    // 6. 转角变化率限幅（保护舵机+防震荡）
    double steering_delta = steering_angle - prev_steering_;
    if (std::abs(steering_delta) > MAX_STEER_RATE * DT) {
        steering_angle = prev_steering_ + std::copysign(MAX_STEER_RATE * DT, steering_delta);
    }
    steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);
    prev_steering_ = steering_angle;
    
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
    // 1. 计算基础预瞄距离
    const double hairpin_lookahead = std::max(1.5 * wheelbase_, L_min_);
    double base_lookahead = L_min_ + k_gain_ * speed;
    double abs_curvature = std::abs(curvature);
    bool cur_sign_negative = (curvature < 0.0);
    if (abs_curvature > 0.03 && cur_sign_negative != prev_sign_negative_) {
        s_bend_lock_frames_ = 25;
    }
    prev_sign_negative_ = cur_sign_negative;

    if (abs_curvature > curvature_threshold_) {
        double curvature_factor = std::sqrt(curvature_threshold_ / abs_curvature);
        base_lookahead = std::max(hairpin_lookahead, base_lookahead * curvature_factor);
    }

    if (abs_curvature > 0.05) {
        base_lookahead = std::min(base_lookahead, 3.0);
    }
    if (abs_curvature > 0.15) {
        base_lookahead = std::min(base_lookahead, 2.0);
    }

    bool is_emergency = (abs_curvature > 0.25);

    if (is_emergency) {
        const double target_lookahead = std::clamp(hairpin_lookahead, L_min_, L_max_);
        const double smoothing_factor = 0.35;
        lookahead_ = lookahead_ * (1.0 - smoothing_factor) + target_lookahead * smoothing_factor;
        emergency_mode_active_ = true;

        if (!emergency_log_printed_) {
            std::cout << "[PurePursuit] 🚨 超急弯！预瞄收敛到 " << lookahead_
                      << "m (目标=" << target_lookahead
                      << "m, 曲率=" << abs_curvature << ")\n";
            emergency_log_printed_ = true;
        }
    } else {
        if (s_bend_lock_frames_ > 0) {
            base_lookahead = std::max(L_min_, wheelbase_ * 1.5);
            lookahead_ = base_lookahead;
            --s_bend_lock_frames_;
            emergency_mode_active_ = true;
        } else {
            double smoothing_factor = emergency_mode_active_ ? 0.08 : 0.12;
            if (emergency_mode_active_) {
                lookahead_ = std::min(
                    lookahead_ * (1.0 - smoothing_factor) + base_lookahead * smoothing_factor,
                    base_lookahead);
            } else {
                lookahead_ = lookahead_ * (1.0 - smoothing_factor) + base_lookahead * smoothing_factor;
            }
            emergency_mode_active_ = false;
        }
        emergency_log_printed_ = false;
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
