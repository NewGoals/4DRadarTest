// Visualizer.hpp - 可视化模�?
#pragma once
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include "SensorData.hpp"

// 可视化基�?
class IVisualizer {
public:
    virtual ~IVisualizer() = default;
    virtual void init() = 0;
    virtual void update(const std::shared_ptr<SensorData>& data) = 0;
    virtual void render() = 0;
};

// 图像可视化器
class ImageVisualizer : public IVisualizer {
private:
    cv::Mat currentFrame;
    const std::string windowName = "Image Viewer";

public:
    void init() override {
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    }

    void update(const std::shared_ptr<SensorData>& data) override {
        auto imageData = std::dynamic_pointer_cast<ImageData>(data);
        if (imageData) {
            currentFrame = imageData->frame.clone();
        }
    }

    void render() override {
        if (!currentFrame.empty()) {
            cv::imshow(windowName, currentFrame);
            cv::waitKey(1);
        }
    }
};

// 点云可视化器
class PointCloudVisualizer : public IVisualizer {
private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud;

public:
    void init() override {
        viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Point Cloud"));
        viewer->setBackgroundColor(0.2, 0.3, 0.3);
        viewer->addCoordinateSystem(1.0, "global");
        viewer->initCameraParameters();
    }

    void update(const std::shared_ptr<SensorData>& data) override {
        auto radarData = std::dynamic_pointer_cast<RadarData>(data);
        if (radarData) {
            currentCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
            for (const auto& point : radarData->points) {
                pcl::PointXYZI pcPoint;
                pcPoint.x = point.x;
                pcPoint.y = point.y;
                pcPoint.z = point.z;
                pcPoint.intensity = point.rcs;
                currentCloud->push_back(pcPoint);
            }
        }
    }

    void render() override {
        if (currentCloud) {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZI>(currentCloud, "radar_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "radar_cloud");
            viewer->spinOnce();
        }
    }
};