#pragma once

#include "base_controller.h"

class PurePursuitController : public BaseController {
private:
    double lookahead_;        // 预瞄距离 (m)
    double wheelbase_;        // 车辆轴距 (m)，用于曲率转转角
    double max_steer_angle_;  // 最大转向角限制 (rad)
    
public:
    // 构造函数：预瞄距离 + 轴距 + 限幅
    PurePursuitController(double lookahead, double wheelbase = 1.5, double max_steer = 0.5);
    
    // 核心算法（必须实现）
    double ComputeSteering(double error, double speed) override;
    
    // 重写接口
    std::string GetName() const override;
    
    // PurePursuit 特有参数：动态调整预瞄距离
    void SetLookahead(double lookahead);
    double GetLookahead() const { return lookahead_; }
};