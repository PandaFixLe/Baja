#include "pid_controller.h"
#include <algorithm>

PIDController::PIDController(double kp, double ki, double kd,
                             double integral_limit, double output_limit)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      integral_(0.0),
      prev_error_(0.0),
      integral_limit_(integral_limit),
      output_limit_(output_limit) {}

double PIDController::Compute(double target, double current, double dt) {
    double error = target - current;
    integral_ += error;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    double derivative = (dt > 1e-6) ? (error - prev_error_) / dt : 0.0;
    prev_error_ = error;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    return std::clamp(output, -output_limit_, output_limit_);
}

std::string PIDController::GetName() const {
    return "Speed_PID_Controller";
}

void PIDController::Reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}

void PIDController::SetGains(double p, double i, double d) {
    kp_ = p;
    ki_ = i;
    kd_ = d;
}
