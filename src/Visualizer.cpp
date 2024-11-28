#include "Visualizer.hpp"

//==============================================================================
// ImageVisualizer 实现
//==============================================================================
ImageVisualizer::ImageVisualizer() {
    try{
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    } catch (const std::runtime_error& e) {
        std::cerr << "Error in ImageVisualizer::init: " << e.what() << std::endl;
    }
}

void ImageVisualizer::update(const std::shared_ptr<SensorData>& data) {
    if (auto imageData = std::dynamic_pointer_cast<ImageData>(data)) {
        currentFrame = imageData->frame.clone();
    }
}

void ImageVisualizer::render() {
    if (!currentFrame.empty()) {
        cv::imshow(windowName, currentFrame);
        cv::waitKey(1);
    }
}

//==============================================================================
// PointCloudVisualizer 实现
//==============================================================================
PointCloudVisualizer::PointCloudVisualizer() {
    try{
        currentCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        viewer.reset(new pcl::visualization::PCLVisualizer(windowName));
        if(!viewer) throw std::runtime_error("Failed to create PCLVisualizer");
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        initialized = true;
    } catch (const std::runtime_error& e) {
        std::cerr << "Error in PointCloudVisualizer::init: " << e.what() << std::endl;
    }
}

void PointCloudVisualizer::update(const std::shared_ptr<SensorData>& data) {
    if(!initialized && !viewer) return;
    if (auto radarData = std::dynamic_pointer_cast<RadarData>(data)) {
        currentCloud->clear();
        for (const auto& point : radarData->points) {
            pcl::PointXYZI p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            p.intensity = point.v_r;
            currentCloud->push_back(p);
        }
    }
}

void PointCloudVisualizer::render() {
    if(!initialized && !viewer){
        std::cerr << "PointCloudVisualizer未初始化" << std::endl;
        return;
    }
    
    if (!currentCloud->empty()) {
        try {
            viewer->removeAllPointClouds();
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_handler(currentCloud, 255, 255, 255);
            viewer->addPointCloud(currentCloud, color_handler, "cloud");
            viewer->spinOnce(1);  // 减小spinOnce的时间
        } catch (const std::exception& e) {
            std::cerr << "渲染点云时发生错误: " << e.what() << std::endl;
        }
    }
    else{
        viewer->spinOnce(1);  // 即使点云为空也需要更新视图
        std::cerr << "当前点云为空" << std::endl;
    }
}

void PointCloudVisualizer::setWindowLayout(int x, int y, int width, int height) {
    viewer->setSize(width, height);
    viewer->setPosition(x, y);
}

void PointCloudVisualizer::setBackgroundColor(float r, float g, float b) {
    viewer->setBackgroundColor(r, g, b);
}

void PointCloudVisualizer::setPointSize(int size) {
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud");
}

void PointCloudVisualizer::getJetColor(float v, uint8_t& r, uint8_t& g, uint8_t& b) {
    float c[4];
    if(v < -1){
        c[0] = 0;
        c[1] = 0;
        c[2] = 1;
    }
    else if(v > 1){
        c[0] = 1;
        c[1] = 0;
        c[2] = 0;
    }
    else{
        c[0] = 0;
        c[1] = 1;
        c[2] = 0;
    }
    r = static_cast<uint8_t>(c[0] * 255);
    g = static_cast<uint8_t>(c[1] * 255);
    b = static_cast<uint8_t>(c[2] * 255);
}

//==============================================================================
// FusionVisualizer 实现
//==============================================================================
void FusionVisualizer::update(const std::shared_ptr<SensorData>& data) {
    imageVis->update(data);
    pointCloudVis->update(data);
}

void FusionVisualizer::render() {
    imageVis->render();
    pointCloudVis->render();
}

void FusionVisualizer::setWindowLayout(int x, int y, int width, int height) {
    // 这里可以根据需要调整两个可视化器的布局
    imageVis->setWindowLayout(x, y, width/2, height);
    pointCloudVis->setWindowLayout(x + width/2, y, width/2, height);
}