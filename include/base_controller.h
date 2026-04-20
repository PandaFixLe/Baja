#pragma once
#include <string>
#include <memory>

class BaseController {
public:
    virtual ~BaseController() = default;
    
    // ✅ 修改：增加 current_heading 参数，Pure Pursuit 和 Stanley 都需要它
    // error: 横向误差 (m)
    // speed: 当前速度 (m/s)
    // curvature: 路径曲率 (1/m)
    // current_heading: 车辆当前航向 (rad)
    virtual double ComputeSteering(double error, double speed, double curvature, double current_heading) = 0;
    
    virtual std::string GetName() const { return "BaseController"; }
    virtual void Init() {}
    virtual void SetGains(double p, double i, double d) {}
};
