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
