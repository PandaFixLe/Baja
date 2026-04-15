#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>
#include "base_controller.h"
#include "pid_controller.h"
#include "pure_pursuit_controller.h"
#include "stanley_controller.h"
#include "../tests/mock_path_generator.h"

// 📊 测试结果结构体
struct TestResult {
    std::string algo_name;       // 算法名称
    double lookahead;           // 预瞄距离 (m)
    double parameter_value;      // 测试参数（如预瞄距离或 Kp）
    double max_error;            // 最大横向误差 (m)
    double avg_error;            // 平均横向误差 (m)
    double avg_steering;         // 平均转向角绝对值 (rad)
    bool oscillation;            // 是否高频震荡
    double steering_jerk;        // 平均转向角变化率 (rad/帧，值越小越平滑)
    double max_steering_util;    // 最大转向角利用率 (0.0~1.0，1.0表示打满)
};

// 🎬 仿真运行函数（核心！）
TestResult RunSimulation(BaseController* controller, MockPathGenerator& mock_gen, double test_speed = 2.0, bool verbose = false) {
    mock_gen.Reset();
    
    // 状态变量
    double current_x = 0.0, current_y = 0.0, current_heading = 0.0;
    double total_error = 0.0, total_steering = 0.0, max_error = 0.0;
    double max_steering_observed = 0.0;
    double prev_steering = 0.0;
    double total_jerk = 0.0; // 累积转向角变化量
    const double MAX_STEER_LIMIT = 0.6; // 实车舵机限幅 (rad)
    
    std::vector<double> steering_history; // 用于震荡检测
    
    if (verbose) {
        std::cout << "\n=== 开始仿真: " << controller->GetName() << " ===\n";
        std::cout << "--------------------------------------------------\n";
    }
    
    const int TOTAL_FRAMES = 250; // 5秒 @ 50Hz
    for (int i = 0; i < TOTAL_FRAMES; ++i) {
        PathPoint ref_point = mock_gen.GetNextPoint();
        
        // 1. 计算误差
        double lateral_error = ref_point.y - current_y;
        
        // 2. 控制器计算
        double steering = controller->ComputeSteering(lateral_error, test_speed);
        
        // 3. 指标统计
        double abs_err = std::abs(lateral_error);
        double abs_steer = std::abs(steering);
        
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
        double yaw_rate = (2.0 / L) * std::tan(steering);
        current_heading += yaw_rate * 0.02;
        current_x += 2.0 * std::cos(current_heading) * 0.02;
        current_y += 2.0 * std::sin(current_heading) * 0.02;
        
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
    
    // 组装返回结果
    TestResult result;
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

// 📋 打印测试结果表格
void PrintResults(const std::vector<TestResult>& results) {
    std::cout << "\n\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                    参数测试结果汇总                           ║\n";
    std::cout << "╠══════════╦══════════════╦══════════════╦════════════╦═══════╣\n";
    std::cout << "║ 算法     ║ 测试参数 ║ 最大误差 ║ 平均误差 ║ 平均转角 ║ 震荡 ║\n";
    std::cout << "╠══════════╬══════════════╬══════════════╬════════════╬═══════╣\n";
    
    for (const auto& r : results) {
        std::cout << "║ " << r.algo_name << std::setw(6) << " ║ " << std::fixed << std::setprecision(2) 
                  << std::setw(8) <<  r.parameter_value << "m  ║ "
                  << std::setw(10) << r.max_error << "  ║ "
                  << std::setw(10) << r.avg_error << "  ║ "
                  << std::setw(8) << r.avg_steering << "rad ║ "
                  << (r.oscillation ? "✅是" : "❌否") 
                  << std::setw(4) << " ║\n";
    }
    
    std::cout << "╚══════════╩══════════════╩══════════════╩════════════╩═══════╝\n";
    
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

// 💾 保存结果到 CSV 文件（可选）
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

int main() {
    std::cout << "🚗 巴哈赛车规控系统 - 参数扫描测试\n";
    std::cout << "================================================\n";
    
    // 1. 创建 Mock 路径生成器
    MockPathGenerator mock_gen;
    mock_gen.GenerateSineWave(1.0, 15.0);  // 振幅 2m，波长 10m
    
// ================= 参数扫描测试 =================
std::vector<TestResult> results;


// 🔹 1. Stanley：扫描横向增益 (Kp)
// 测试不同的 Stanley 增益参数
std::vector<double> kp_values = {0.8, 1.0, 1.2, 1.5, 2.0};

for (double kp : kp_values) {
    std::cout << "\n🔧 测试 Stanley Kp增益: " << kp << "\n";
    
    auto controller = std::make_unique<StanleyController>(
        kp,     // k_p 横向误差增益
        1.0,    // k_theta 航向误差增益
        0.6     // 最大转角
    );
    
    TestResult result = RunSimulation(controller.get(), mock_gen, false);
    result.algo_name = "Stanley_Controller";
    result.parameter_value = kp;  // ✅ 这一行必须有！
    results.push_back(result);
    
    std::cout << "   最大误差: " << result.max_error << "m, "
              << "平均误差: " << result.avg_error << "m\n";
}

// 🔹 2. Pure Pursuit：扫描预瞄距离 (Ld)
// 继续测试 Pure Pursuit（用于对比）
std::vector<double> lookahead_values = {1.0, 1.5, 2.0, 2.5, 3.0};

for (double Ld : lookahead_values) {
    std::cout << "\n🔧 测试 Pure Pursuit 预瞄距离: " << Ld << "m\n";
    mock_gen.Reset();
    auto controller = std::make_unique<PurePursuitController>(
        Ld, 1.5, 0.6
    );
    
    TestResult result = RunSimulation(controller.get(), mock_gen, false);
    result.algo_name = "PurePursuit_Controller";
    result.parameter_value = Ld;  // ✅ 设置参数值
    results.push_back(result);
    
    std::cout << "   最大误差: " << result.max_error << "m\n";
}
// ================= 动态预瞄随车速变化测试 =================
std::cout << "\n🚀 测试动态预瞄随车速变化效果\n";
std::vector<double> test_speeds = {1.0, 2.0, 3.0, 4.0}; // m/s
auto pp_controller = std::make_unique<PurePursuitController>(2.0, 1.5, 0.6); // 初始预瞄值会被动态逻辑覆盖

for (double v : test_speeds) {
    std::cout << "\n🔧 测试车速: " << v << " m/s\n";
    
    TestResult res = RunSimulation(pp_controller.get(), mock_gen, v, false);
    res.algo_name = "PurePursuit_DynamicLd";
    res.parameter_value = v;  // 记录测试车速
    results.push_back(res);
    
    std::cout << "   平均误差: " << res.avg_error << "m | Jerk: " << res.steering_jerk << "\n";
}
// 3. 输出结果
PrintResults(results);
SaveResultsToCSV(results, "build/parameter_test_results.csv");
PrintAlgorithmComparison(results);

// 🏆 改进的推荐逻辑：多指标综合评分
auto best_it = std::min_element(results.begin(), results.end(),
    [](const TestResult& a, const TestResult& b) {
        // 综合评分 = 平均误差 * 1.0 + 平均转角 * 0.3 + (震荡 ? 2.0 : 0)
        double score_a = a.avg_error * 1.0 + a.avg_steering * 0.3 + (a.oscillation ? 2.0 : 0);
        double score_b = b.avg_error * 1.0 + b.avg_steering * 0.3 + (b.oscillation ? 2.0 : 0);
        return score_a < score_b;
    });
    std::cout << "\n🏆 推荐参数: 预瞄距离 = " << best_it->parameter_value << "m\n";
    std::cout << "   理由: 平均误差最小 (" << best_it->avg_error << "m)\n";
   // 5. 推荐最佳参数
std::cout << "\n🏆 推荐参数:\n";
std::cout << "   算法: " << best_it->algo_name << "\n";

if (best_it->algo_name == "Stanley_Controller") {
    std::cout << "   Kp增益 = " << best_it->parameter_value << "\n";
} else {
    std::cout << "   预瞄距离 = " << best_it->parameter_value << "m\n";
}

std::cout << "   理由: 综合评分最优 (平均误差=" << best_it->avg_error 
          << "m, Jerk=" << best_it->steering_jerk << "rad/frame)\n";
// 6. 详细演示最佳参数
std::cout << "\n🏆 详细演示最佳参数表现：\n";
std::cout << "   算法: " << best_it->algo_name << "\n";
std::cout << "   参数: " << best_it->parameter_value << "\n";
std::cout << "----------------------------------------\n";

if (best_it->algo_name == "Stanley_Controller") {
    // 演示 Stanley
    auto best_controller = std::make_unique<StanleyController>(
        best_it->parameter_value,  // Kp
        1.0,    // k_theta
        0.6     // 最大转角
    );
    RunSimulation(best_controller.get(), mock_gen, true);
} else {
    // 演示 Pure Pursuit
    auto best_controller = std::make_unique<PurePursuitController>(
        best_it->parameter_value,  // Ld
        1.5,    // 轴距
        0.6     // 最大转角
    );
    RunSimulation(best_controller.get(), mock_gen, true);
}
    return 0;
}