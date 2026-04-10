#pragma once  // 防止重复包含（比 #ifndef 更简洁）

#include <string>
#include <memory>

class BaseController {
public:
    virtual ~BaseController() = default;
    
    // 纯虚函数：所有控制器必须实现
    virtual double ComputeSteering(double error, double speed) = 0;
    
    // 虚函数：可选重写
    virtual std::string GetName() const { return "BaseController"; }
    virtual void Init() { /* 默认空实现 */ }
    
    // 参数接口：基类提供默认行为
    virtual void SetGains(double p, double i, double d) {
        // 空实现：非 PID 算法可忽略
    }
};