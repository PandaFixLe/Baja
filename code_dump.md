### 📄 `include/base_controller.h`
#pragma once  // 防止重复包含（比 #ifndef 更简洁）

#include <string>
#include <memory>

class BaseController {
public:
    virtual ~BaseController() = default;
    
    // 纯虚函数：所有控制器必须实现
    virtual double ComputeSteering(double error, double speed, double curvature = 0.0) = 0;
    
    // 虚函数：可选重写
    virtual std::string GetName() const { return "BaseController"; }
    virtual void Init() { /* 默认空实现 */ }
    
    // 参数接口：基类提供默认行为
    virtual void SetGains(double p, double i, double d) {
        // 空实现：非 PID 算法可忽略
    }
};
---

### 📄 `include/pid_controller.h`
#pragma once

#include "base_controller.h"

class PIDController : public BaseController {
private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;  // PID 需要状态
    
public:
    PIDController(double kp, double ki, double kd);
    
    // 核心算法（必须实现）
    double ComputeSteering(double error, double speed, double curvature = 0.0) override;
    
    // 重写接口
    std::string GetName() const override;
    void SetGains(double p, double i, double d) override;
};
---

### 📄 `include/pure_pursuit_controller.h`
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
    double ComputeSteering(double error, double speed, double curvature = 0.0) override;
    
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
---

### 📄 `include/stanley_controller.h`
#pragma once
#include "base_controller.h"

class StanleyController : public BaseController {
private:
    double k_p_;          // 横向误差增益
    double k_theta_;      // 航向误差增益
    double max_steer_;    // 最大转向角
public:
    StanleyController(double kp = 1.0, double ktheta = 1.0, double max_steer = 0.6);
    double ComputeSteering(double lateral_error, double speed, double curvature = 0.0) override;
    std::string GetName() const override { return "Stanley_Controller"; }
};
---

### 📄 `src/main.cpp`
// src/main.cpp - 精简优化版
#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

#include "base_controller.h"
#include "pid_controller.h"
#include "pure_pursuit_controller.h"
#include "stanley_controller.h"
#include "../tests/mock_path_generator.h"

// 📊 测试结果结构体（保持不变）
struct TestResult {
    std::string algo_name;
    double parameter_value;
    double max_error, avg_error, avg_steering;
    bool oscillation;
    double steering_jerk, max_steering_util;

    // 综合评分：误差*1.0 + jerk*0.5 + 震荡惩罚*2.0
    double GetScore() const {
        return avg_error * 1.0 + steering_jerk * 0.5 + (oscillation ? 2.0 : 0);
    }
};

// 🎬 仿真运行函数（保持不变，核心逻辑）
TestResult RunSimulation(BaseController* controller, MockPathGenerator& mock_gen,
                        double test_speed, bool verbose = false,
                        std::vector<std::tuple<double,double,double>>* trajectory = nullptr) {
    mock_gen.Reset();

    // 状态变量 - 从路径中间开始测试，让车有初始误差
    double current_x = 5.0;  // 从x=5m开始
    double current_y = 1.0;  // 有1m初始误差
    double current_heading = 0.0;
    double total_error = 0.0, total_steering = 0.0, max_error = 0.0;
    double max_steering_observed = 0.0;
    double prev_steering = 0.0;
    double total_jerk = 0.0; // 累积转向角变化量
    const double MAX_STEER_LIMIT = 0.6; // 实车舵机限幅 (rad)

    std::vector<double> steering_history; // 用于震荡检测
    int large_error_count = 0;           // 连续超限误差计数，确保每次仿真重置

    if (verbose) {
        std::cout << "\n=== 开始仿真: " << controller->GetName() << " ===\n";
        std::cout << "--------------------------------------------------\n";
    }

    const int TOTAL_FRAMES = 1000; // 20秒 @ 50Hz
    for (int i = 0; i < TOTAL_FRAMES; ++i) {
        // 1. 获取当前的预瞄距离 (如果是 PurePursuit)
        double lookahead_dist = 2.0; // 默认值
        auto* pp_ctrl = dynamic_cast<PurePursuitController*>(controller);
        if (pp_ctrl) lookahead_dist = pp_ctrl->GetLookahead();

        // 2. ✅ 关键修改：根据车的当前位置找目标点
        // 注意：这里需要传入车的 current_x, current_y
        PathPoint ref_point = mock_gen.GetTargetPoint(current_x, current_y, lookahead_dist);

        // 3. 计算横向误差：沿参考路径法线投影
        double dx = ref_point.x - current_x;
        double dy = ref_point.y - current_y;
        double path_yaw = ref_point.heading;
        double lateral_error = -std::sin(path_yaw) * dx + std::cos(path_yaw) * dy;

        // 【放在这里】如果是PurePursuit，动态更新参数
        if (pp_ctrl && verbose) {
            std::cout << "[Frame " << i << "] 当前预瞄距离: "
                      << pp_ctrl->GetLookahead() << " m, 曲率: "
                      << ref_point.curvature << " 1/m" << std::endl;
        }

        // 4. 控制器计算
        double steering = controller->ComputeSteering(
            lateral_error,
            test_speed,
            ref_point.curvature
        );

        // 3. 指标统计
        double abs_err = std::abs(lateral_error);
        double abs_steer = std::abs(steering);

        // 🔹 优先级3：添加"误差超限保护"（规则 F2.7 鲁棒性要求）
        // 如果连续 3 帧误差 > 2.5m（接近出界），触发保护
        if (abs_err > 2.5) {
            large_error_count++;
            if (large_error_count >= 3 && verbose) {
                std::cout << "⚠️ 警告：连续大误差，建议检查参数或感知模块\n";
            }
        } else {
            large_error_count = 0;
        }

        total_error += abs_err;
        total_steering += abs_steer;
        if (abs_err > max_error) max_error = abs_err;
        if (abs_steer > max_steering_observed) max_steering_observed = abs_steer;

        // 转向角变化率 (Jerk)
        double jerk = std::abs(steering - prev_steering);
        total_jerk += jerk;
        prev_steering = steering;

        // 记录历史用于震荡检测
        steering_history.push_back(steering);
        if (steering_history.size() > 10) {
            steering_history.erase(steering_history.begin());
        }

        // 4. 车辆运动学更新（自行车模型）
        double L = 1.5; // 轴距
        double yaw_rate = (test_speed / L) * std::tan(steering);
        current_heading += yaw_rate * 0.02;
        current_x += test_speed * std::cos(current_heading) * 0.02;
        current_y += test_speed * std::sin(current_heading) * 0.02;

        // 📍 收集轨迹数据（如果提供了轨迹指针）
        if (trajectory) {
            trajectory->emplace_back(current_x, current_y, current_heading);
        }

        // 5. 详细日志输出
        if (verbose && i % 10 == 0) {
            std::cout << "t=" << (i * 0.02) << "s | "
                      << "参考=(" << ref_point.x << ", " << ref_point.y << ") | "
                      << "误差=" << lateral_error << "m | "
                      << "转角=" << steering << "rad\n";
        }
    }

    if (verbose) {
        std::cout << "--------------------------------------------------\n";
        std::cout << "✅ 仿真完成！\n";
    }

    // 震荡检测：最近10帧内符号变化 > 4次 判定为高频震荡
    int sign_changes = 0;
    for (size_t i = 1; i < steering_history.size(); ++i) {
        if (steering_history[i] * steering_history[i-1] < 0) sign_changes++;
    }
    TestResult result;
    // 新增：如果连续出现多次符号翻转，记录为“严重震荡”
    bool severe_oscillation = (sign_changes > 6);
    result.oscillation = severe_oscillation;

    // 组装返回结果
    result.algo_name = controller->GetName();
    result.parameter_value = 0.0; // 由调用方赋值
    result.max_error = max_error;
    result.avg_error = total_error / TOTAL_FRAMES;
    result.avg_steering = total_steering / TOTAL_FRAMES;
    result.oscillation = (sign_changes > 4);
    result.steering_jerk = total_jerk / TOTAL_FRAMES;          // 平均每帧转向角跳变量
    result.max_steering_util = max_steering_observed / MAX_STEER_LIMIT; // 限幅利用率

    return result;
}

// 📋 打印结果表格（更整齐的对齐样式）
void PrintResults(const std::vector<TestResult>& results) {
    const int kAlgoWidth = 14;
    const int kParamWidth = 10;
    const int kMaxWidth = 10;
    const int kAvgWidth = 10;
    const int kSteerWidth = 10;
    const int kOscWidth = 8;

    std::cout << "\n\n";
    std::cout << "+" << std::string(kAlgoWidth, '=') << "+"
              << std::string(kParamWidth, '=') << "+"
              << std::string(kMaxWidth, '=') << "+"
              << std::string(kAvgWidth, '=') << "+"
              << std::string(kSteerWidth, '=') << "+"
              << std::string(kOscWidth, '=') << "+\n";
    std::cout << " | " << std::left << std::setw(kAlgoWidth - 1) << "算法"
              << " | " << std::left << std::setw(kParamWidth - 1) << "测试参数"
              << " | " << std::left << std::setw(kMaxWidth - 1) << "最大误差"
              << " | " << std::left << std::setw(kAvgWidth - 1) << "平均误差"
              << " | " << std::left << std::setw(kSteerWidth - 1) << "平均转角"
              << " | " << std::left << std::setw(kOscWidth - 1) << "震荡"
              << "|\n";
    std::cout << "+" << std::string(kAlgoWidth, '=') << "+"
              << std::string(kParamWidth, '=') << "+"
              << std::string(kMaxWidth, '=') << "+"
              << std::string(kAvgWidth, '=') << "+"
              << std::string(kSteerWidth, '=') << "+"
              << std::string(kOscWidth, '=') << "+\n";

    for (const auto& r : results) {
        std::ostringstream param_ss;
        param_ss << std::fixed << std::setprecision(2) << r.parameter_value << "m";

        std::ostringstream steer_ss;
        steer_ss << std::fixed << std::setprecision(2) << r.avg_steering << "rad";

        std::cout << "║ " << std::left << std::setw(kAlgoWidth - 1) << r.algo_name
                  << "║ " << std::right << std::setw(kParamWidth - 1) << param_ss.str()
                  << "║ " << std::right << std::setw(kMaxWidth - 1) << std::fixed << std::setprecision(2) << r.max_error
                  << "║ " << std::right << std::setw(kAvgWidth - 1) << std::fixed << std::setprecision(2) << r.avg_error
                  << "║ " << std::right << std::setw(kSteerWidth - 1) << steer_ss.str()
                  << "║ " << std::right << std::setw(kOscWidth - 1) << (r.oscillation ? "✅是" : "❌否")
                  << "║\n";
    }

    std::cout << "+" << std::string(kAlgoWidth, '=') << "+"
              << std::string(kParamWidth, '=') << "+"
              << std::string(kMaxWidth, '=') << "+"
              << std::string(kAvgWidth, '=') << "+"
              << std::string(kSteerWidth, '=') << "+"
              << std::string(kOscWidth, '=') << "+\n";
}
// 算法对比总结
void PrintAlgorithmComparison(const std::vector<TestResult>& results) {
    double pp_avg_error = 0, pp_avg_jerk = 0;
    double stanley_avg_error = 0, stanley_avg_jerk = 0;
    int pp_count = 0, stanley_count = 0;

    for (const auto& r : results) {
        if (r.algo_name == "PurePursuit_Controller") {
            pp_avg_error += r.avg_error;
            pp_avg_jerk += r.steering_jerk;
            pp_count++;
        } else if (r.algo_name == "Stanley_Controller") {
            stanley_avg_error += r.avg_error;
            stanley_avg_jerk += r.steering_jerk;
            stanley_count++;
        }
    }

    if (pp_count > 0 && stanley_count > 0) {
        pp_avg_error /= pp_count;
        pp_avg_jerk /= pp_count;
        stanley_avg_error /= stanley_count;
        stanley_avg_jerk /= stanley_count;

        std::cout << "\n📊 算法对比总结：\n";
        std::cout << "   Pure Pursuit:  平均误差=" << pp_avg_error
                  << "m, Jerk=" << pp_avg_jerk << "\n";
        std::cout << "   Stanley:       平均误差=" << stanley_avg_error
                  << "m, Jerk=" << stanley_avg_jerk << "\n";

        if (stanley_avg_error < pp_avg_error * 0.9) {
            std::cout << "   → Stanley 在跟踪精度上优势明显\n";
        } else if (pp_avg_jerk < stanley_avg_jerk * 0.9) {
            std::cout << "   → Pure Pursuit 在控制平滑性上优势明显\n";
        } else {
            std::cout << "   → 两种算法性能接近，需根据实车调优\n";
        }
    }
}

// 💾 保存CSV（保持不变）
void SaveResultsToCSV(const std::vector<TestResult>& results, const std::string& filename) {
    std::ofstream file(filename);
    file << "algo,parameter,max_error,avg_error,avg_steering,oscillation,steering_jerk,max_steering_util\n";
for (const auto& r : results) {
    file << r.algo_name << ","
         << r.parameter_value << ","
         << r.max_error << ","
         << r.avg_error << ","
         << r.avg_steering << ","
         << (r.oscillation ? "1" : "0") << ","
         << r.steering_jerk << ","
         << r.max_steering_util << "\n";
}
    file.close();
    std::cout << "📄 结果已保存到: " << filename << "\n";
}

// 🗺️ 新增函数：保存轨迹数据
void SaveTrajectoryToCSV(const std::string& filename,
                        const std::vector<PathPoint>& ref_path,
                        const std::vector<std::tuple<double,double,double>>& car_traj) {
    std::string full_path = "/home/pandafixle/projects/baja_controller/" + filename;
    std::ofstream file(full_path);
    if (!file.is_open()) {
        std::cerr << "❌ 无法创建文件: " << full_path << "\n";
        return;
    }
    file << "type,x,y,heading\n";

    for (const auto& pt : ref_path) {
        file << "ref," << pt.x << "," << pt.y << "," << pt.heading << "\n";
    }
    for (const auto& [x,y,h] : car_traj) {
        file << "car," << x << "," << y << "," << h << "\n";
    }
    file.close();
    std::cout << "📊 轨迹数据已保存到: " << full_path << "\n";
}

// 🏆 查找最佳结果（合并重复逻辑）
const TestResult* FindBestResult(const std::vector<TestResult>& results) {
    if (results.empty()) return nullptr;
    return &(*std::min_element(results.begin(), results.end(),
        [](const TestResult& a, const TestResult& b) {
            return a.GetScore() < b.GetScore();
        }));
}

// 🔧 创建控制器工厂（简化演示逻辑）
std::unique_ptr<BaseController> CreateController(const std::string& name, double param) {
    if (name == "Stanley_Controller") {
        return std::make_unique<StanleyController>(param, 1.0, 0.6);
    } else {
        auto ctrl = std::make_unique<PurePursuitController>(2.0, 1.5, 0.6);
        ctrl->SetDynamicParameters(0.3, 1.0, 4.0);
        ctrl->SetCurvatureParameters(0.15, 0.8);
        return ctrl;
    }
}

// 🧪 参数扫描测试（模块化）
std::vector<TestResult> RunParameterScan() {
    std::vector<TestResult> results;
    MockPathGenerator mock_gen;
    mock_gen.GenerateCompetitionTrack();  // ✅ 使用组合路径

    std::vector<double> test_params = {1.0, 1.5, 2.0, 2.5, 3.0}; // 预瞄距离或Kp

    for (double param : test_params) {
        // Pure Pursuit 测试
        auto pp = std::make_unique<PurePursuitController>(param, 1.5, 0.6);
        pp->SetDynamicParameters(0.3, 1.0, 4.0);
        pp->SetCurvatureParameters(0.15, 0.8);

        TestResult res = RunSimulation(pp.get(), mock_gen, 3.0, false);
        res.algo_name = "PurePursuit";
        res.parameter_value = param;
        results.push_back(res);

        // Stanley 测试（可选）
        auto stanley = std::make_unique<StanleyController>(param, 1.0, 0.6);
        res = RunSimulation(stanley.get(), mock_gen, 3.0, false);
        res.algo_name = "Stanley";
        res.parameter_value = param;
        results.push_back(res);
    }
    return results;
}

// 🚀 动态预瞄效果验证（模块化）
void RunDynamicLookaheadTest() {
    std::cout << "\n🔍 动态预瞄效果验证（不同速度+曲率）\n";

    MockPathGenerator mock_gen;
    mock_gen.GenerateCompetitionTrack();

    auto controller = std::make_unique<PurePursuitController>(2.0, 1.5, 0.6);
    controller->SetDynamicParameters(0.3, 1.0, 4.0);
    controller->SetCurvatureParameters(0.15, 0.8);

    std::vector<double> speeds = {1.0, 2.5, 4.0};

    for (double v : speeds) {
        mock_gen.Reset();
        TestResult res = RunSimulation(controller.get(), mock_gen, v, false);
        std::cout << "📊 v=" << v << "m/s → 平均误差:" << res.avg_error
                  << "m | Jerk:" << res.steering_jerk
                  << (res.oscillation ? " ⚠️震荡" : " ✅稳定") << "\n";
    }
}

// 🎯 主函数：流程清晰，只负责调度
int main() {
    std::cout << "🏎️ 智能巴哈赛车规控 - 精简测试框架\n";
    std::cout << "=====================================\n\n";

    // 1️⃣ 参数扫描
    std::cout << "📋 Step 1: 参数扫描测试\n";
    auto results = RunParameterScan();
    PrintResults(results);
    SaveResultsToCSV(results, "build/scan_results.csv");

    // 2️⃣ 动态预瞄验证
    std::cout << "\n📋 Step 2: 动态预瞄效果验证\n";
    RunDynamicLookaheadTest();

    // 3️⃣ 推荐最佳参数
    std::cout << "\n📋 Step 3: 推荐最佳参数\n";
    const TestResult* best = FindBestResult(results);
    if (best) {
        std::cout << "🏆 推荐: " << best->algo_name
                  << " | 参数=" << best->parameter_value
                  << " | 综合评分=" << best->GetScore() << "\n";
        std::cout << "   理由: 误差=" << best->avg_error << "m, Jerk=" << best->steering_jerk << "\n";

        // 4️⃣ 详细演示最佳参数
        std::cout << "\n🎬 详细演示最佳参数:\n";
        MockPathGenerator demo_gen;
        demo_gen.GenerateCompetitionTrack();
        auto demo_ctrl = CreateController(best->algo_name, best->parameter_value);
        
        // 📊 收集轨迹数据用于可视化
        std::vector<std::tuple<double,double,double>> trajectory;
        RunSimulation(demo_ctrl.get(), demo_gen, 3.0, false, &trajectory);
        
        // 💾 保存轨迹数据到 CSV
        SaveTrajectoryToCSV("build/trajectory.csv", demo_gen.GetAllPoints(), trajectory);
    }

    std::cout << "\n✅ 测试完成！结果已保存到 build/ 目录\n";
    return 0;
}
---

### 📄 `src/pid_controller.cpp`
#include "pid_controller.h"
#include <iostream>

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0), prev_error_(0) {}

double PIDController::ComputeSteering(double error, double speed, double curvature) {
    (void)curvature;
    // 简化版 PID（实际需加积分限幅、微分滤波）
    integral_ += error;
    double derivative = error - prev_error_;
    prev_error_ = error;
    
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    std::cout << "[PID] output=" << output << "\n";
    return output;
}

std::string PIDController::GetName() const {
    return "PID_Controller";
}

void PIDController::SetGains(double p, double i, double d) {
    kp_ = p; ki_ = i; kd_ = d;
    std::cout << "[PID] 参数更新: " << kp_ << "/" << ki_ << "/" << kd_ << "\n";
}
---

### 📄 `src/pure_pursuit_controller.cpp`
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

double PurePursuitController::ComputeSteering(double error, double speed, double curvature) {
    // 1. 根据速度和曲率动态调整预瞄距离 (这个保留，用于调参)
    UpdateParameters(speed, curvature);
    
    double dynamic_lookahead = lookahead_;
    
    // 2. ✅ 修正核心算法：Pure Pursuit 必须基于几何误差计算
    // 公式：k = 2 * lateral_error / L^2
    // 注意：error 是有符号的，决定了向左还是向右转
    
    // 删除之前的 if (curvature != 0.0) 判断！
    // 强制使用反馈控制公式
    double path_curvature = (2.0 * error) / (dynamic_lookahead * dynamic_lookahead);
    
    // 3. 计算转向角：delta = atan(L * k)
    double steering_angle = std::atan2(wheelbase_ * path_curvature, 1.0);
    
    // 4. 转向限幅
    steering_angle = std::clamp(steering_angle, -max_steer_angle_, max_steer_angle_);
    
    // 调试输出（可选）
    // std::cout << "[PP] error=" << error << " L=" << dynamic_lookahead 
    //           << " -> steer=" << steering_angle << "\n";
    
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
    // 1. 计算目标预瞄距离（保持你原有的逻辑）
    double base_lookahead = L_min_ + k_gain_ * speed;
    double abs_curvature = std::abs(curvature);
    
    if (abs_curvature > curvature_threshold_) {
        double curvature_factor = curvature_threshold_ / abs_curvature;
        base_lookahead = std::max(emergency_lookahead_, base_lookahead * curvature_factor);
    }
    base_lookahead = std::clamp(base_lookahead, L_min_, L_max_);

    // 🔹 新增：判断是否处于“紧急模式”（超急弯）
    bool is_emergency = (abs_curvature > 0.25);

    if (is_emergency) {
        // 超急弯：强制缩短预瞄距离，并标记为紧急状态
        lookahead_ = std::max(0.6, base_lookahead * 0.5);
        emergency_mode_active_ = true;
        
        // ✅ 修改：只在变化较大时打印日志，避免重复
        if (std::abs(lookahead_ - base_lookahead) > 0.3 && !emergency_log_printed_) {
            std::cout << "[PurePursuit] 🚨 超急弯！预瞄紧急缩短至" << lookahead_ << "m (曲率=" << abs_curvature << ")\n";
            emergency_log_printed_ = true;  // 标记已打印
        }
    } else {
        // 恢复阶段：缓慢增加预瞄距离，避免突变
        if (emergency_mode_active_) {
            // 渐进恢复：每次只增加 10%
            double recovery_factor = 0.1;
            double target_lookahead = lookahead_ * (1 - recovery_factor) + base_lookahead * recovery_factor;
            lookahead_ = std::min(target_lookahead, base_lookahead); // 不超过正常值
        } else {
            // 正常情况：使用低通滤波（20%权重）
            double smoothing_factor = 0.2;
            double target_lookahead = lookahead_ * (1 - smoothing_factor) + base_lookahead * smoothing_factor;
            lookahead_ = target_lookahead;
    
        }

        // 只有变化较大时才打印日志
        if (std::abs(lookahead_ - base_lookahead) > 0.3) {
            std::cout << "[PurePursuit] ⚠️ 预瞄大幅调整: " << base_lookahead << "m -> " << lookahead_ << "m\n";
        }

        emergency_mode_active_ = false;
        emergency_log_printed_ = false;  // 重置标志位
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
---

### 📄 `src/stanley_controller.cpp`
#include "stanley_controller.h"
#include <cmath>
#include <algorithm>

StanleyController::StanleyController(double kp, double ktheta, double max_steer)
    : k_p_(kp), k_theta_(ktheta), max_steer_(max_steer) {}

double StanleyController::ComputeSteering(double lateral_error, double speed, double curvature) {
    (void)curvature;
    // 🛡️ 防零速除错
    double v_safe = std::max(speed, 0.5); 
    
    // Stanley 核心公式：δ = θ_e + atan2(k*e / v, 1)
    // 此处简化假设航向误差 θ_e ≈ 0（实际需从感知获取路径切线角）
    double theta_e = 0.0; 
    
    double delta = theta_e + std::atan2(k_p_ * lateral_error / v_safe, 1.0);
    return std::clamp(delta, -max_steer_, max_steer_);
}
---

### 📄 `tests/mock_path_generator.cpp`
#include "mock_path_generator.h"
#include <iostream>
#include <cmath>

MockPathGenerator::MockPathGenerator() 
    : current_idx_(0), current_x_(0.0), current_y_(0.0), current_heading_(0.0) {
    std::cout << "[MockPathGenerator] 初始化完成\n";
}

MockPathGenerator::~MockPathGenerator() {}

void MockPathGenerator::Clear() {
    path_.clear();
    current_idx_ = 0;
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_heading_ = 0.0;
}

void MockPathGenerator::Reset() {
    current_idx_ = 0;
}

bool MockPathGenerator::IsFinished() const {
    return current_idx_ >= path_.size();
}

PathPoint MockPathGenerator::GetNextPoint() {
    if (current_idx_ < path_.size()) {
        return path_[current_idx_++];
    }
    // 返回最后一个点
    if (!path_.empty()) {
        return path_.back();
    }
    return PathPoint();
}

// 在 mock_path_generator.cpp 中实现
PathPoint MockPathGenerator::GetTargetPoint(double car_x, double car_y, double lookahead_dist) {
    // 1. 找到距离车身最近的点
    size_t closest_idx = 0;
    double min_dist = 1e9;
    for (size_t i = 0; i < path_.size(); ++i) {
        double d = std::hypot(path_[i].x - car_x, path_[i].y - car_y);
        if (d < min_dist) {
            min_dist = d;
            closest_idx = i;
        }
    }

    // 2. 从最近点开始，沿路径向前累积距离，直到达到 lookahead_dist
    double accumulated_dist = 0;
    for (size_t i = closest_idx; i < path_.size() - 1; ++i) {
        double seg_len = std::hypot(path_[i+1].x - path_[i].x, path_[i+1].y - path_[i].y);
        accumulated_dist += seg_len;
        
        if (accumulated_dist >= lookahead_dist) {
            return path_[i]; // 找到目标点
        }
    }
    return path_.back(); // 如果路径不够长，返回终点
}

void MockPathGenerator::AddPoint(double x, double y, double heading, double curvature, double speed_limit) {
    double timestamp = path_.empty() ? 0.0 : path_.back().timestamp + dt_;
    path_.emplace_back(x, y, heading, curvature, speed_limit, timestamp);
}

double MockPathGenerator::CalculateHeading(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

double MockPathGenerator::CalculateCurvature(double radius) {
    if (std::abs(radius) < 1e-6) {
        return 0.0;
    }
    return 1.0 / radius;
}

void MockPathGenerator::GenerateStraightLine(double length) {
    path_.clear();
    current_idx_ = 0;
    
    double num_points = length / 0.5;  // 每 0.5m 一个点
    for (int i = 0; i <= num_points; ++i) {
        PathPoint pt;
        pt.x = i * 0.5;
        pt.y = 0.0;
        pt.heading = 0.0;
        pt.curvature = 0.0;
        pt.speed_limit = 5.0;
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成直线路径，长度=" << length << "m, 点数=" << path_.size() << "\n";
}

void MockPathGenerator::GenerateCircle(double radius) {
    path_.clear();
    current_idx_ = 0;
    
    int num_points = 100;  // 100 个点画圆
    double angle_step = 2 * M_PI / num_points;
    
    for (int i = 0; i < num_points; ++i) {
        PathPoint pt;
        double angle = i * angle_step;
        pt.x = radius * std::cos(angle);
        pt.y = radius * std::sin(angle);
        pt.heading = angle + M_PI / 2;  // 切线方向
        pt.curvature = 1.0 / radius;
        pt.speed_limit = 2.0;
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成圆形路径，半径=" << radius << "m, 点数=" << path_.size() << "\n";
}

void MockPathGenerator::GenerateSineWave(double amplitude, double wavelength) {
    path_.clear();
    current_idx_ = 0;
    
    double total_length = wavelength * 2;  // 两个周期
    int num_points = static_cast<int>(total_length / 0.5);
    
    for (int i = 0; i < num_points; ++i) {
        PathPoint pt;
        double x = i * 0.5;
        double y = amplitude * std::sin(2 * M_PI * x / wavelength);
        
        pt.x = x;
        pt.y = y;
        pt.heading = std::atan2(amplitude * 2 * M_PI / wavelength * std::cos(2 * M_PI * x / wavelength), 1.0);
        pt.curvature = 0.05;  // 简化曲率
        pt.speed_limit = 3.0;
        pt.timestamp = i * dt_;
        path_.push_back(pt);
    }
    
    std::cout << "[Mock] 生成S弯路径，振幅=" << amplitude << "m, 波长=" << wavelength << "m\n";
}

void MockPathGenerator::GenerateSineWaveAppend(double amplitude, double wavelength) {
    double total_length = wavelength * 2;  // 两个周期
    int num_points = static_cast<int>(total_length / 0.5);
    
    for (int i = 0; i < num_points; ++i) {
        double s = i * 0.5;  // 沿路径的距离
        
        // 计算横向偏移
        double offset = amplitude * std::sin(2 * M_PI * s / wavelength);
        
        // 前进方向的单位向量
        double dir_x = std::cos(current_heading_);
        double dir_y = std::sin(current_heading_);
        
        // 垂直方向的单位向量（向左）
        double perp_x = -dir_y;
        double perp_y = dir_x;
        
        // 位置计算
        double x = current_x_ + s * dir_x + offset * perp_x;
        double y = current_y_ + s * dir_y + offset * perp_y;
        
        // 计算航向角（切线方向）
        double dy_ds = amplitude * 2 * M_PI / wavelength * std::cos(2 * M_PI * s / wavelength);
        double heading = current_heading_ + std::atan2(dy_ds, 1.0);
        
        // 计算曲率
        double d2y_ds2 = -amplitude * std::pow(2 * M_PI / wavelength, 2) * 
                         std::sin(2 * M_PI * s / wavelength);
        double curvature = d2y_ds2 / std::pow(1 + dy_ds * dy_ds, 1.5);
        
        AddPoint(x, y, heading, curvature, 3.0);
    }
    
    // 更新终点位置（S弯结束时offset回到0，所以只前进total_length）
    current_x_ += total_length * std::cos(current_heading_);
    current_y_ += total_length * std::sin(current_heading_);
}
void MockPathGenerator::GenerateStraight(double length, double speed_limit) {
    double step = 0.5;
    int num_points = static_cast<int>(length / step);
    
    for (int i = 0; i <= num_points; ++i) {
        double x = current_x_ + i * step * std::cos(current_heading_);
        double y = current_y_ + i * step * std::sin(current_heading_);
        AddPoint(x, y, current_heading_, 0.0, speed_limit);
    }
    
    current_x_ += length * std::cos(current_heading_);
    current_y_ += length * std::sin(current_heading_);
}

void MockPathGenerator::GenerateCurve(double radius, double angle_deg, bool clockwise, double speed_limit) {
    double angle_rad = angle_deg * M_PI / 180.0;
    int num_points = static_cast<int>(std::abs(angle_deg) / 2.0);
    double angle_step = angle_rad / num_points;
    
    double direction = clockwise ? -1.0 : 1.0;
    double center_x = current_x_ - direction * radius * std::sin(current_heading_);
    double center_y = current_y_ + direction * radius * std::cos(current_heading_);
    
    double start_angle = std::atan2(current_y_ - center_y, current_x_ - center_x);
    
    for (int i = 0; i <= num_points; ++i) {
        double current_angle = start_angle + direction * i * angle_step;
        double x = center_x + radius * std::cos(current_angle);
        double y = center_y + radius * std::sin(current_angle);
        double heading = current_angle + (clockwise ? -M_PI/2 : M_PI/2);
        
        AddPoint(x, y, heading, direction / radius, speed_limit);
    }
    
    current_x_ = center_x + radius * std::cos(start_angle + direction * angle_rad);
    current_y_ = center_y + radius * std::sin(start_angle + direction * angle_rad);
    current_heading_ += direction * angle_rad;
}

void MockPathGenerator::GenerateCompetitionTrack() {
    Clear();
    std::cout << "=== 生成比赛赛道 ===\n";
    
    GenerateStraight(20.0, 4.0);
    GenerateSineWaveAppend(12.0, 25.0);
    GenerateCurve(15.0, 90.0, true, 3.5);
    GenerateStraight(15.0, 4.0);
    GenerateCurve(8.0, 180.0, true, 1.5);
    GenerateStraight(30.0, 6.0);
    
    std::cout << "赛道生成完成！总点数：" << path_.size() << "\n";
}

void MockPathGenerator::GenerateHairpinTestSection() {
    Clear();
    std::cout << "=== 生成发卡弯测试段 ===\n";
    
    GenerateStraight(15.0, 2.0);
    GenerateCurve(6.0, 180.0, true, 1.2);
    GenerateStraight(5.0, 2.0);
    GenerateCurve(10.0, 180.0, false, 1.8);
    
    std::cout << "发卡弯测试段完成！\n";
}

void MockPathGenerator::GenerateHighSpeedSection() {
    Clear();
    std::cout << "=== 生成高速循迹测试段 ===\n";
    
    GenerateStraight(20.0, 5.0);
    GenerateCurve(30.0, 45.0, true, 6.0);
    GenerateStraight(30.0, 7.0);
    
    std::cout << "高速循迹测试段完成！\n";
}

void MockPathGenerator::GenerateOffroadObstacles() {
    Clear();
    std::cout << "=== 生成越野障碍段 ===\n";
    
    GenerateStraight(10.0, 3.0);
    
    // 模拟搓板路
    for (int i = 0; i < 50; ++i) {
        double x = current_x_ + i * 0.2 * std::cos(current_heading_);
        double y = current_y_ + i * 0.2 * std::sin(current_heading_);
        AddPoint(x, y, current_heading_, 0.0, 2.0);
    }
    current_x_ += 10.0 * std::cos(current_heading_);
    current_y_ += 10.0 * std::sin(current_heading_);
    
    GenerateSineWave(8.0, 15.0);
    
    std::cout << "越野障碍段完成！\n";
}

---

### 📄 `tests/mock_path_generator.h`
#ifndef MOCK_PATH_GENERATOR_H
#define MOCK_PATH_GENERATOR_H

#include <vector>
#include <string>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct PathPoint {
    double x;
    double y;
    double heading;
    double curvature;
    double speed_limit;
    double timestamp;
    
    PathPoint() : x(0), y(0), heading(0), curvature(0), speed_limit(3.0), timestamp(0) {}
    PathPoint(double x_, double y_, double h_, double c_, double v_, double t = 0)
        : x(x_), y(y_), heading(h_), curvature(c_), speed_limit(v_), timestamp(t) {}
};

class MockPathGenerator {
public:
    MockPathGenerator();
    ~MockPathGenerator();
    
    PathPoint GetTargetPoint(double car_x, double car_y, double lookahead_dist);

    void Clear();
    PathPoint GetNextPoint();
    const std::vector<PathPoint>& GetAllPoints() const { return path_; }
    void Reset();
    size_t GetTotalPoints() const { return path_.size(); }
    bool IsFinished() const;
    
    // 基本路径生成
    void GenerateStraightLine(double length);
    void GenerateCircle(double radius);
    void GenerateSineWave(double amplitude, double wavelength);
    void GenerateSineWaveAppend(double amplitude, double wavelength);
    void GenerateStraight(double length, double speed_limit = 5.0);
    void GenerateCurve(double radius, double angle_deg, bool clockwise = true, double speed_limit = 3.0);
    
    // 组合路径
    void GenerateCompetitionTrack();
    void GenerateHairpinTestSection();
    void GenerateHighSpeedSection();
    void GenerateOffroadObstacles();
    
private:
    std::vector<PathPoint> path_;
    size_t current_idx_;
    double current_x_;
    double current_y_;
    double current_heading_;
    
    static constexpr double dt_ = 0.02;  // 20ms @ 50Hz
    
    void AddPoint(double x, double y, double heading, double curvature, double speed_limit);
    double CalculateHeading(double x1, double y1, double x2, double y2);
    double CalculateCurvature(double radius);
};

#endif // MOCK_PATH_GENERATOR_H

---

### 📄 `tests/test_pure_pursuit.cpp`
#include <gtest/gtest.h>
#include "../include/pure_pursuit_controller.h"
#include <cmath>

// 测试 1：零误差输出应为 0
TEST(PurePursuitTest, ZeroErrorOutput) {
    PurePursuitController pp(2.0, 1.5, 0.6);
    double out = pp.ComputeSteering(0.0, 2.0);
    EXPECT_NEAR(out, 0.0, 1e-6);
}

// 测试 2：正误差产生正转角，且在限幅内
TEST(PurePursuitTest, PositiveErrorAndClamp) {
    PurePursuitController pp(1.0, 1.5, 0.5); // 小预瞄+小限幅，易触发饱和
    double out = pp.ComputeSteering(2.0, 2.0);
    EXPECT_GT(out, 0.0);
    EXPECT_LE(out, 0.5); // 不超过 max_steer
}

// 测试 3：速度为 0 时不除零崩溃
TEST(PurePursuitTest, ZeroSpeedSafe) {
    PurePursuitController pp(2.0, 1.5, 0.6);
    double out = pp.ComputeSteering(1.0, 0.0);
    EXPECT_FALSE(std::isnan(out));
    EXPECT_FALSE(std::isinf(out));
}
---

