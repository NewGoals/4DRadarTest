#include "Visualizer.hpp"

//==============================================================================
// ImageVisualizer ʵ��
//==============================================================================
ImageVisualizer::ImageVisualizer() {
    try{
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
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
// PointCloudVisualizer ʵ��
//==============================================================================
PointCloudVisualizer::PointCloudVisualizer() {
    // �ڹ��캯����Ԥ�ȼ���������
    for (int i = 0; i < 11; ++i) {
        float x = -25.0f + i * 5.0f;
        float y = -25.0f + i * 5.0f;
        
        // ��ֱ��
        gridLines.push_back({
            pcl::PointXYZ(x, -25.0f, 0),
            pcl::PointXYZ(x, 25.0f, 0)
        });
        
        // ˮƽ��
        gridLines.push_back({
            pcl::PointXYZ(-25.0f, y, 0),
            pcl::PointXYZ(25.0f, y, 0)
        });
    }
    try{
        currentCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        viewer.reset(new pcl::visualization::PCLVisualizer(windowName));
        if(!viewer) throw std::runtime_error("Failed to create PCLVisualizer");
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        // �������
        for (size_t i = 0; i < gridLines.size(); ++i) {
            viewer->addLine(
                gridLines[i].first, 
                gridLines[i].second, 
                0.5, 0.5, 0.5, 
                "grid_line_" + std::to_string(i)
            );
        }

        // ��������ӽ�
        viewer->setCameraPosition(
            0.0, -50.0, 50.0,   // ���λ�� (x, y, z)
            0.0, 0.0, 0.0,      // �۲�Ŀ��� 
            0.0, 0.0, 1.0       // �Ϸ�������
        );

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
            pcl::PointXYZRGB p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            
            uint8_t r, g, b;
            getJetColor(point.v_r, r, g, b);
            p.r = r;
            p.g = g;
            p.b = b;

            currentCloud->push_back(p);
        }
    }
}

void PointCloudVisualizer::render() {
    if(!initialized && !viewer){
        std::cerr << "PointCloudVisualizerδ��ʼ��" << std::endl;
        return;
    }

    if (!currentCloud->empty()) {
        viewer->removeAllPointClouds();
        viewer->addPointCloud(currentCloud, "cloud");
    }
    else {
        std::cerr << "��ǰ����Ϊ��" << std::endl;
    }
    
    viewer->spinOnce(0);
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
    if(v < -0.5){
        c[0] = 0;
        c[1] = 0;
        c[2] = 1;
    }
    else if(v > 0.5){
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
// FusionVisualizer ʵ��
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
    // ������Ը�����Ҫ�����������ӻ����Ĳ���
    imageVis->setWindowLayout(x, y, width/2, height);
    pointCloudVis->setWindowLayout(x + width/2, y, width/2, height);
}