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

// 雷达数据类
class RadarData : public SensorData {
public:
    std::vector<RadarPoint> points;
};
