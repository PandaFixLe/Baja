#pragma once

#include "base_controller.h"

class PurePursuitController : public BaseController {
private:
    double lookahead_;          // 预瞄距离 (m)
    double wheelbase_;          // 车辆轴距 (m)，用于曲率转转角
    double max_steer_angle_;    // 最大转向角限制 (rad)
    
    // 动态预瞄参数
    double k_gain_;             // 速度增益系数
    double L_min_;              // 最小预瞄距离
    double L_max_;              // 最大预瞄距离
    
    // 曲率相关参数
    double curvature_threshold_;  // 曲率阈值（用于检测急弯）
    double emergency_lookahead_;  // 急弯时的紧急预瞄距离
    bool emergency_mode_active_;  // 是否处于超急弯状态，用于抑制重复日志输出
    bool emergency_log_printed_;

public:
    // 构造函数：预瞄距离 + 轴距 + 限幅
    PurePursuitController(double lookahead, double wheelbase = 1.5, double max_steer = 0.6);
    
    // 核心算法（必须实现）
    // 新增：curvature参数用于应对发卡弯
    double ComputeSteering(double error, double speed, double curvature = 0.0,double current_heading= 0.0) override;
    
    // 重写接口
    std::string GetName() const override;
    
    // PurePursuit 特有参数：动态调整预瞄距离
    void SetLookahead(double lookahead);
    double GetLookahead() const { return lookahead_; }
    
    // 新增：动态更新参数接口（根据速度和曲率）
    void UpdateParameters(double speed, double curvature);
    
    // 新增：设置动态预瞄参数
    void SetDynamicParameters(double k_gain, double L_min, double L_max);
    
    // 新增：设置曲率相关参数
    void SetCurvatureParameters(double threshold, double emergency_lookahead);
    
    // Getter方法
    double GetKGain() const { return k_gain_; }
    double GetLMin() const { return L_min_; }
    double GetLMax() const { return L_max_; }
};