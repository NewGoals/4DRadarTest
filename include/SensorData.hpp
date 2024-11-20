// SensorData.hpp - ���ݽṹ����
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

// �״�����ݽṹ
struct RadarPoint {
    float x, y, z;
    float rcs;
    float v_r;
};

// ���������ݻ���
class SensorData {
public:
    virtual ~SensorData() = default;
    int64_t timestamp;  // ʹ�� int64_t �洢����ʱ���
};

// ͼ��������
class ImageData : public SensorData {
public:
    cv::Mat frame;
};

// �״�������
class RadarData : public SensorData {
public:
    std::vector<RadarPoint> points;
};
