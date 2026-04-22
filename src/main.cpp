// src/main.cpp - 原始完整版本（最小修改）
#include <iostream>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <tuple>

#include "base_controller.h"
#include "pid_controller.h"
#include "pure_pursuit_controller.h"
#include "stanley_controller.h"
#include "../tests/mock_path_generator.h"

struct TestResult {
    std::string algo_name;
    double parameter_value;
    double max_error, avg_error, avg_steering;
    bool oscillation;
    double steering_jerk, max_steering_util;

    double GetScore() const {
        return avg_error * 1.0 + steering_jerk * 0.5 + (oscillation ? 2.0 : 0);
    }
};

// ✅ 替换原有的 RunSimulation 函数
TestResult RunSimulation(BaseController* controller, MockPathGenerator& mock_gen, 
                        double test_speed, bool verbose = false, 
                        std::vector<std::tuple<double, double, double>>* trajectory = nullptr) {
    mock_gen.Reset();

    // 状态变量
    double current_x = 0.0, current_y = 0.0, current_heading = 0.0;
    double current_speed = 0.0;
    double total_error = 0.0, total_steering = 0.0, max_error = 0.0;
    double max_steering_observed = 0.0;
    double prev_steering = 0.0;
    double total_jerk = 0.0;
    const double MAX_STEER_LIMIT = 0.6; 

    std::vector<double> steering_history;

    if (verbose) {
        std::cout << "\n=== 开始仿真: " << controller->GetName() << " ===\n";
    }

    const auto& full_path = mock_gen.GetAllPoints();
    const double dt = 0.02;
    double track_length = 0.0;
    for (size_t i = 1; i < full_path.size(); ++i) {
        track_length += std::hypot(
            full_path[i].x - full_path[i - 1].x,
            full_path[i].y - full_path[i - 1].y);
    }
    const double min_safe_speed = 1.5;
    const int MAX_FRAMES = static_cast<int>((track_length / (min_safe_speed * dt)) * 1.3);
    int frames_run = 0;
    PIDController speed_pid(1.4, 0.15, 0.05, 8.0, 2.5);

    for (int i = 0; i < MAX_FRAMES; ++i) {
        frames_run = i + 1;
        // 1. 获取预瞄距离
        double lookahead_dist = 2.0;
        auto* pp_ctrl = dynamic_cast<PurePursuitController*>(controller);
        if (pp_ctrl) lookahead_dist = pp_ctrl->GetLookahead();

        // 2. 获取最近路径点和目标点
        size_t closest_index = mock_gen.GetClosestIndex(current_x, current_y);
        PathPoint closest_point = full_path[closest_index];
        PathPoint ref_point = mock_gen.GetTargetPoint(
            current_x, current_y, current_heading, lookahead_dist);

        // 3. 计算目标点相对车辆的局部坐标
        double dx = ref_point.x - current_x;
        double dy = ref_point.y - current_y;
        double target_local_x = dx * std::cos(current_heading) + dy * std::sin(current_heading);
        double target_local_y = -dx * std::sin(current_heading) + dy * std::cos(current_heading);

        // 仍然保留最近路径点坐标系误差用于统计观察
        double closest_dx = closest_point.x - current_x;
        double closest_dy = closest_point.y - current_y;
        double precise_lateral_error = -closest_dx * std::sin(closest_point.heading)
                                       + closest_dy * std::cos(closest_point.heading);

        double controller_error = precise_lateral_error;
        if (dynamic_cast<PurePursuitController*>(controller)) {
            controller_error = target_local_y;
        }

        double heading_error = std::atan2(
            std::sin(closest_point.heading - current_heading),
            std::cos(closest_point.heading - current_heading));
        if (pp_ctrl) {
            controller_error += 0.8 * lookahead_dist * heading_error;
        }

        double curvature_speed_limit = std::clamp(
            test_speed - 3.0 * std::abs(closest_point.curvature), 1.5, test_speed);
        double target_speed = std::min(closest_point.speed_limit, curvature_speed_limit);
        target_speed = std::clamp(target_speed, 0.8, test_speed);
        double speed_command = speed_pid.Compute(target_speed, current_speed, dt);
        current_speed += speed_command * dt;
        current_speed = std::clamp(current_speed, 0.0, test_speed);

        // 4. 控制器计算
        double steering = controller->ComputeSteering(
            controller_error,
            current_speed,
            closest_point.curvature,
            current_heading
        );

        // 5. 统计指标
        double abs_err = std::abs(precise_lateral_error);
        double abs_steer = std::abs(steering);
        total_error += abs_err;
        total_steering += abs_steer;
        if (abs_err > max_error) max_error = abs_err;
        if (abs_steer > max_steering_observed) max_steering_observed = abs_steer;

        double jerk = std::abs(steering - prev_steering);
        total_jerk += jerk;
        prev_steering = steering;

        steering_history.push_back(steering);
        if (steering_history.size() > 10) steering_history.erase(steering_history.begin());

        // 6. ✅ 关键：记录轨迹 (在位置更新后记录)
        if (trajectory) {
            trajectory->emplace_back(current_x, current_y, current_heading);
        }

        // 7. 车辆运动学更新
        double L = 1.5;
        double yaw_rate = (current_speed / L) * std::tan(steering);
        current_heading += yaw_rate * dt;
        current_x += current_speed * std::cos(current_heading) * dt;
        current_y += current_speed * std::sin(current_heading) * dt;

        // 8. 日志输出
        if (verbose && i % 50 == 0) {
            std::cout << "t=" << (i * 0.02)
                      << "s | CTE=" << precise_lateral_error
                      << "m | LocalY=" << target_local_y
                      << "m | LocalX=" << target_local_x
                      << "m | HeadingErr=" << heading_error
                      << "rad | Speed=" << current_speed
                      << "m/s | TargetSpeed=" << target_speed
                      << "m/s | Steer=" << steering << "rad\n";
        }

        if (closest_index >= mock_gen.GetTotalPoints() - 15 && i > 100) {
            if (verbose) {
                std::cout << "[RunSimulation] 路径进度到达终点附近，提前结束。index="
                          << closest_index << "/" << full_path.size() << "\n";
            }
            break;
        }
    }

    // 震荡检测
    int sign_changes = 0;
    for (size_t i = 1; i < steering_history.size(); ++i) {
        if (steering_history[i] * steering_history[i-1] < 0) sign_changes++;
    }

    TestResult result;
    result.algo_name = controller->GetName();
    result.parameter_value = 0.0;
    result.max_error = max_error;
    result.avg_error = total_error / frames_run;
    result.avg_steering = total_steering / frames_run;
    result.oscillation = (sign_changes > 4);
    result.steering_jerk = total_jerk / frames_run;
    result.max_steering_util = max_steering_observed / MAX_STEER_LIMIT;

    return result;
}

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
    std::cout << "📄 结果已保存到：" << filename << "\n";
}

void SaveTrajectoryToCSV(const std::string& filename,
                        const std::vector<PathPoint>& ref_path,
                        const std::vector<std::tuple<double,double,double>>& car_traj) {
    std::string full_path = "/home/pandafixle/projects/baja_controller/" + filename;
    std::ofstream file(full_path);
    if (!file.is_open()) {
        std::cerr << "❌ 无法创建文件：" << full_path << "\n";
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
    std::cout << "📊 轨迹数据已保存到：" << full_path << "\n";
}

const TestResult* FindBestResult(const std::vector<TestResult>& results) {
    if (results.empty()) return nullptr;
    return &(*std::min_element(results.begin(), results.end(),
        [](const TestResult& a, const TestResult& b) {
            return a.GetScore() < b.GetScore();
        }));
}

std::unique_ptr<BaseController> CreateController(const std::string& name, double param) {
    if (name == "Stanley_Controller" || name == "Stanley") {
        return std::make_unique<StanleyController>(param, 1.0, 0.6);
    } else {
        auto ctrl = std::make_unique<PurePursuitController>(param, 1.5, 0.6);
        ctrl->SetDynamicParameters(0.3, 1.0, 4.0);
        ctrl->SetCurvatureParameters(0.15, 0.8);
        return ctrl;
    }
}

std::vector<TestResult> RunParameterScan() {
    std::vector<TestResult> results;
    MockPathGenerator mock_gen;
    mock_gen.GenerateCompetitionTrack();

    std::vector<double> test_params = {1.0, 1.5, 2.0, 2.5, 3.0};

    for (double param : test_params) {
        auto pp = std::make_unique<PurePursuitController>(param, 1.5, 0.6);
        pp->SetDynamicParameters(0.3, 1.0, 4.0);
        pp->SetCurvatureParameters(0.15, 0.8);

        TestResult res = RunSimulation(pp.get(), mock_gen, 3.0, false);
        res.algo_name = "PurePursuit_Controller";
        res.parameter_value = param;
        results.push_back(res);

        auto stanley = std::make_unique<StanleyController>(param, 1.0, 0.6);
        res = RunSimulation(stanley.get(), mock_gen, 3.0, false);
        res.algo_name = "Stanley_Controller";
        res.parameter_value = param;
        results.push_back(res);
    }
    return results;
}

void RunDynamicLookaheadTest() {
    std::cout << "\n🔍 动态预瞄效果验证（不同速度 + 曲率）\n";

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

int main() {
    std::cout << "🏎️ 智能巴哈赛车规控 - 精简测试框架\n";
    std::cout << "=====================================\n\n";

    std::cout << "📋 Step 1: 参数扫描测试\n";
    auto results = RunParameterScan();
    PrintResults(results);
    SaveResultsToCSV(results, "build/scan_results.csv");

    std::cout << "\n📋 Step 2: 动态预瞄效果验证\n";
    RunDynamicLookaheadTest();

    std::cout << "\n📋 Step 3: 推荐最佳参数\n";
    const TestResult* best = FindBestResult(results);
    if (best) {
        std::cout << "🏆 推荐：" << best->algo_name
                  << " | 参数=" << best->parameter_value
                  << " | 综合评分=" << best->GetScore() << "\n";
        std::cout << "   理由：误差=" << best->avg_error << "m, Jerk=" << best->steering_jerk << "\n";

        std::cout << "\n🎬 详细演示最佳参数:\n";
        MockPathGenerator demo_gen;
        demo_gen.GenerateCompetitionTrack();
        auto demo_ctrl = CreateController(best->algo_name, best->parameter_value);
        
        std::vector<std::tuple<double,double,double>> trajectory;
        RunSimulation(demo_ctrl.get(), demo_gen, 3.0, false, &trajectory);
        
        SaveTrajectoryToCSV("build/trajectory.csv", demo_gen.GetAllPoints(), trajectory);
    }

    std::cout << "\n✅ 测试完成！结果已保存到 build/ 目录\n";
    return 0;
}
