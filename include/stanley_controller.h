#pragma once
#include "base_controller.h"

class StanleyController : public BaseController {
private:
    double k_p_;          // 横向误差增益
    double k_theta_;      // 航向误差增益
    double max_steer_;    // 最大转向角
public:
    StanleyController(double kp = 1.0, double ktheta = 1.0, double max_steer = 0.6);
    double ComputeSteering(double lateral_error, double speed) override;
    std::string GetName() const override { return "Stanley_Controller"; }
};