#include <iostream>
#include <memory>
#include "base_controller.h"
#include "pid_controller.h"
#include "pure_pursuit_controller.h"

int main() {
    std::unique_ptr<BaseController> controller = 
        std::make_unique<PIDController>(0.5, 0.1, 0.05);
    
    std::cout << ">>> 算法: " << controller->GetName() << "\n";
    controller->Init();
    controller->ComputeSteering(0.2, 2.0);
    
    // 切换算法
    controller = std::make_unique<PurePursuitController>(2.0);
    std::cout << ">>> 算法: " << controller->GetName() << "\n";
    controller->ComputeSteering(0.2, 2.0);
    
    return 0;
}