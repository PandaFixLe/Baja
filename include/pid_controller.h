#pragma once

#include <string>

class PIDController {
private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;
    double integral_limit_;
    double output_limit_;

public:
    PIDController(double kp, double ki, double kd,
                  double integral_limit = 5.0,
                  double output_limit = 3.0);

    double Compute(double target, double current, double dt);
    void Reset();
    void SetGains(double p, double i, double d);
    std::string GetName() const;
};
