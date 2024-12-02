// Visualizer.hpp - 可视化模块
#pragma once

// Windows 和编译器相关定义
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

#include <memory>
#include <unordered_map>
#include <string>

// PCL放在最后
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

// 数据结构头文件，其中包含opencv
#include "SensorData.hpp"

// 可视化基类
class IVisualizer {
public:
    virtual ~IVisualizer() = default;
    virtual void update(const std::shared_ptr<SensorData>& data) = 0;
    virtual void render() = 0;

    // 添加窗口布局控制
    virtual void setWindowLayout(int x, int y, int width, int height) {};
};

// 图像可视化器
class ImageVisualizer : public IVisualizer {
private:
    cv::Mat currentFrame;
    const std::string windowName = "Image Viewer";
    cv::Rect windowLayout{0, 0, 800, 600};  // 默认布局

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

// 点云可视化器
class PointCloudVisualizer : public IVisualizer {
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud;
    std::string windowName = "Point Cloud";
    bool initialized = false;
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> gridLines;

    void getJetColor(float v, uint8_t& r, uint8_t& g, uint8_t& b);
public:
    PointCloudVisualizer();
    void update(const std::shared_ptr<SensorData>& data) override;
    void render() override;
    void setWindowLayout(int x, int y, int width, int height) override;

    // 点云特有的设置
    void setBackgroundColor(float r, float g, float b);
    void setPointSize(int size);
};

// 融合可视化器
class FusionVisualizer : public IVisualizer {
private:
    std::shared_ptr<ImageVisualizer> imageVis;
    std::shared_ptr<PointCloudVisualizer> pointCloudVis;
    cv::Mat projectionMatrix;  // 点云到图像的投影矩阵

public:
    FusionVisualizer(){
        imageVis = std::make_shared<ImageVisualizer>();
        pointCloudVis = std::make_shared<PointCloudVisualizer>();
    }
    void update(const std::shared_ptr<SensorData>& data) override;
    void render() override;
    void setWindowLayout(int x, int y, int width, int height) override;

    // 设置投影矩阵
    void setProjectionMatrix(const cv::Mat& matrix){
        projectionMatrix = matrix.clone();
    }
};
