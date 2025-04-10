// SensorData.hpp - 数据结构定义
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

// 雷达点数据结构
struct RadarPoint {
    float x, y, z;
    float rcs;
    float v_r;
    float ana_snr;
};

// 雷达点迹数据结构
struct RadarTrace {
    uint32_t ID;
    uint32_t type;
    float x_speed;
    float y_speed;
    float z_speed;
    float x_axes;
    float y_axes;
    float z_axes;
    float length;
    float azimuth_angle;
    float elevation_angle;
    float SNR;
    float Peak_energy;
};

// 传感器数据基类
class SensorData {
public:
    virtual ~SensorData() = default;
    int64_t timestamp;  // 使用 int64_t 存储毫秒时间戳
};

// 图像数据类
class ImageData : public SensorData {
public:
    cv::Mat frame;
};

// 雷达数据点云类
class RadarData : public SensorData {
public:
    std::vector<RadarPoint> points;
};

class RadarTraceData : public SensorData {
public:
    std::vector<RadarTrace> traces;
};
