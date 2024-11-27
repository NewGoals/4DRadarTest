// Visualizer.hpp - ���ӻ�ģ��
#pragma once

// Windows �ͱ�������ض���
#ifdef _WIN32
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef _CRT_SECURE_NO_WARNINGS
        #define _CRT_SECURE_NO_WARNINGS
    #endif
#endif

// ��׼��
#include <memory>
#include <unordered_map>
#include <string>

// Eigen
#define EIGEN_NO_DEBUG  // ���õ�����Ϣ
#define EIGEN_USE_BLAS  // ���ʹ��BLAS��ȷ������
#include <Eigen/Dense>

// OpenCV ͷ�ļ�
#include <opencv2/opencv.hpp>

// PCL�������
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "SensorData.hpp"

// ���ӻ�����
class IVisualizer {
public:
    virtual ~IVisualizer() = default;
    virtual void update(const std::shared_ptr<SensorData>& data) = 0;
    virtual void render() = 0;

    // ��Ӵ��ڲ��ֿ���
    virtual void setWindowLayout(int x, int y, int width, int height) {};
};

// ͼ����ӻ���
class ImageVisualizer : public IVisualizer {
private:
    cv::Mat currentFrame;
    const std::string windowName = "Image Viewer";
    cv::Rect windowLayout{0, 0, 800, 600};  // Ĭ�ϲ���

public:
    ImageVisualizer();
    void update(const std::shared_ptr<SensorData>& data) override;
    void render() override;
    void setWindowLayout(int x, int y, int width, int height) override {
        windowLayout = cv::Rect(x, y, width, height);
        cv::moveWindow(windowName, x, y);
        cv::resizeWindow(windowName, width, height);
    }
};

// ���ƿ��ӻ���
class PointCloudVisualizer : public IVisualizer {
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;
    std::string windowName = "Point Cloud";
    bool initialized = false;
public:
    PointCloudVisualizer();
    void update(const std::shared_ptr<SensorData>& data) override;
    void render() override;
    void setWindowLayout(int x, int y, int width, int height) override;

    // �������е�����
    void setBackgroundColor(float r, float g, float b);
    void setPointSize(int size);
};

// �ںϿ��ӻ���
class FusionVisualizer : public IVisualizer {
private:
    std::shared_ptr<ImageVisualizer> imageVis;
    std::shared_ptr<PointCloudVisualizer> pointCloudVis;
    cv::Mat projectionMatrix;  // ���Ƶ�ͼ���ͶӰ����

public:
    FusionVisualizer(){
        imageVis = std::make_shared<ImageVisualizer>();
        pointCloudVis = std::make_shared<PointCloudVisualizer>();
    }
    void update(const std::shared_ptr<SensorData>& data) override;
    void render() override;
    void setWindowLayout(int x, int y, int width, int height) override;

    // ����ͶӰ����
    void setProjectionMatrix(const cv::Mat& matrix){
        projectionMatrix = matrix.clone();
    }
};
