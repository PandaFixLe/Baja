#pragma once

#include "base_controller.h"

class PIDController : public BaseController {
private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;  // PID 需要状态
    
public:
    PIDController(double kp, double ki, double kd);
    
    // 核心算法（必须实现）
    double ComputeSteering(double error, double speed) override;
    
    // 重写接口
    std::string GetName() const override;
    void SetGains(double p, double i, double d) override;
};