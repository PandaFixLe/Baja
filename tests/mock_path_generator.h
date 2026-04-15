#pragma once

#include <vector>
#include <cmath>

// 路径点结构（模拟感知组输出的参考路径）
struct PathPoint {
    double x;           // 全局 X 坐标 (m)
    double y;           // 全局 Y 坐标 (m)
    double heading;     // 航向角 (rad)
    double timestamp;   // 时间戳 (s)
};

class MockPathGenerator {
private:
    std::vector<PathPoint> path_;
    size_t current_idx_ = 0;
    double dt_ = 0.02;  // 仿真步长 20ms (50Hz)
    
    
public:
    MockPathGenerator();
    
    // 生成不同类型路径
    void GenerateStraightLine(double length = 20.0);     // 直线
    void GenerateCircle(double radius = 5.0);            // 圆形
    void GenerateSineWave(double amplitude = 2.0, double wavelength = 10.0); // 正弦波
    
    // 获取下一个路径点（模拟实时数据流）
    PathPoint GetNextPoint();
    
    // 重置到起点
    
    void Reset() { current_idx_ = 0; } 
    // 是否到达终点
    bool IsFinished() const;
};