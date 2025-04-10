#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include "SensorData.hpp"
#include "DataReaderFactory.hpp"
#include "DisplayManager.hpp" // 其中既有eigen又有opencv
#include "InferenceModel.hpp"

namespace fs = std::filesystem;


bool calibrateCameraFromImages(const std::string &folder_path, cv::Size board_size, float square_size, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);
// 外参标定
std::vector<cv::Mat> calib();

//==============================================================================
// 通用映射类
//==============================================================================
class RadarImageMapper
{
private:
    cv::Mat camera_matrix_; // 相机内参矩阵
    cv::Mat dist_coeffs_;   // 畸变系数
    cv::Mat rvec_;          // 旋转向量
    cv::Mat tvec_;          // 平移向量

    bool isValidPoint(const RadarPoint &pt);

public:
    // 初始化标定参数
    bool init(const std::string &calib_file, const std::string &extrinsic_file);
    void setExtrinsicParam(cv::Mat rvec, cv::Mat tvec);
    // 生成深度掩膜（包含深度信息）
    cv::Mat createDepthMask(const std::vector<RadarPoint> &radar_points,
                            const cv::Size &image_size,
                            float max_depth = 300.0f,
                            bool use_nearest = true);
    // 计算框的深度值（取区域中心点附近区域的平均深度）
    float calculateBoxDepth(const cv::Rect& box, const cv::Mat& depth_mask);
    // 根据检测框和深度估算物体尺寸
    cv::Vec3f estimateObjectSize(const cv::Rect& box, float depth, int class_id);
    // 添加立方体
    void addBoundingBoxToViewer(pcl::visualization::PCLVisualizer::Ptr viewer,
        const cv::Point3f& center,
        const cv::Vec3f& size,
        const std::string& id);
    // 渲染检测物
    void visualizeDetections(pcl::visualization::PCLVisualizer::Ptr viewer,const std::vector<InferenceModel::DetectionResult>& results,const cv::Mat& depth_mask); 
    // 图像投影
    cv::Mat mapperProject(const std::vector<RadarPoint> &radar_points,
        cv::Mat &image);
    // 坐标转换：图像坐标到雷达坐标
    cv::Point3f imageToRadar(const cv::Point2f &image_point, float depth);

public:
    // 获取参数
    std::vector<cv::Mat> getCalibParam();

public:
    // 雷达到图像的映射操作通用函数
    std::vector<cv::Point2f> radarToImage(const std::vector<cv::Point3f>& radar_point);
    cv::Mat cubeBottomMapper(cv::Mat image, float x_min, float y_min, float z_min, float x_max, float y_max, float z_max, float z_real);
};

//==============================================================================
// 显示控制类
//==============================================================================
void projectRadarPoints(const std::vector<RadarPoint> &radar_points,
                        cv::Mat &image,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &dist_coeffs,
                        const cv::Mat &rvec,
                        const cv::Mat &tvec);

class CalibRadarVisualizer
{
private:
    cv::Mat camera_matrix_, dist_coeffs_;
    cv::Mat rvec_, tvec_, source_rvec, source_tvec;
    std::shared_ptr<RadarFileReader> radar_reader_;
    std::shared_ptr<ImageFileReader> image_reader_;
    DisplayManager display_manager_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    pcl::visualization::PCLVisualizer::Ptr viewer_source_;

    //映射操作
    YOLOv6ONNX model = YOLOv6ONNX(L"E:/Source/4DRadarTest/models/last_ckpt.onnx");
    RadarImageMapper mapper;
    std::vector<InferenceModel::DetectionResult> results;
    cv::Mat depthMask;

    // DBSCAN parameters with default values
    float eps_ = 0.5;
    int min_pts_ = 5;

    // GUI controls
    bool pause_ = true;
    bool params_changed_ = false;

    struct ExtrinsicParams
    {
        double rx = 0, ry = 0, rz = 0; // rotation in degrees
        double tx = 0, ty = 0, tz = 0; // translation in meters
    } extrinsic_params_;

    // 重置外参到初始值
    void resetExtrinsicParams();

    // 保存当前外参到文件
    void saveExtrinsicParams();

    // 键盘回调函数
    void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *);

    void initializeVisualizers();

    // 创建控制面板
    void createControlPanel();

    void updateExtrinsicParams();

    // 刷新当前帧
    void processCurrentFrame();

    // 跳转下一帧
    void processNextFrame();

    void processRadarData(const std::shared_ptr<RadarData> &radar_data);

    void processImageData(const std::shared_ptr<ImageData> &image_data,
                          const std::shared_ptr<RadarData> &radar_data);

public:
    CalibRadarVisualizer(const std::string &calib_file,
                         const std::string &extrinsic_file,
                         const std::string &radar_path,
                         const std::string &image_path);

    void run();
};
