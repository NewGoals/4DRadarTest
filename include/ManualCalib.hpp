#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

bool calibrateCameraFromImages(const std::string& folder_path, cv::Size board_size, float square_size, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {
    std::vector<cv::String> image_paths;
    for (const auto& entry : fs::directory_iterator(folder_path)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".bmp") {
            image_paths.push_back(entry.path().string());
        }
    }

    if (image_paths.empty()) {
        std::cerr << "未找到任何图片文件！" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    // 生成棋盘格的三维坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_size.height; ++i) {
        for (int j = 0; j < board_size.width; ++j) {
            obj.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    // 检测棋盘格角点
    for (const auto& path : image_paths) {
        cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);

        
        std::vector<cv::Point2f> corners;

        bool found = cv::findChessboardCorners(image, board_size, corners);
        if (found) {
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            image_points.push_back(corners);
            object_points.push_back(obj);
        }
    }

    if (image_points.empty()) {
        std::cerr << "未检测到有效的棋盘格角点！" << std::endl;
        return false;
    }

    // 进行相机标定
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(object_points, image_points, cv::Size(image_points[0][0].x, image_points[0][0].y),
                                     camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "标定完成，重投影误差: " << rms << std::endl;
    std::cout << "相机内参矩阵:\n" << camera_matrix << std::endl;
    std::cout << "畸变系数:\n" << dist_coeffs << std::endl;

    // 保存内参矩阵和畸变系数到文件
    cv::FileStorage fs("camera_calibration.yml", cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "camera_matrix" << camera_matrix;
        fs << "dist_coeffs" << dist_coeffs;
        fs.release();
        std::cout << "内参矩阵和畸变系数已保存到 camera_calibration.yml" << std::endl;
    } else {
        std::cerr << "无法保存内参矩阵和畸变系数！" << std::endl;
        return false;
    }

    // 验证图像
    for (const auto& path : image_paths) {
        cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
        // 获取原始图像的尺寸
        int original_width = image.cols;
        int original_height = image.rows;

        // 计算缩放后的尺寸
        int new_width = original_width / 4;  // 宽度缩小
        int new_height = original_height / 4; // 高度缩小

        // // 缩放图像
        // cv::Mat resized_image;
        // cv::resize(image, resized_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

        // 校正图像
        cv::Mat undistorted_image;
        cv::undistort(image, undistorted_image, camera_matrix, dist_coeffs);

        // 显示原始图像和校正后的图像
        cv::Mat resized_image;
        cv::Mat resized_undistorted_image;

        // 调整原始图像大小
        cv::resize(image, resized_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

        // 调整校正后的图像大小
        cv::resize(undistorted_image, resized_undistorted_image, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

        // 创建一个空白图像用于存放拼接后的结果
        cv::Mat combined_image;

        // 上下拼接两张图像
        cv::vconcat(resized_image, resized_undistorted_image, combined_image);

        // 显示拼接后的图像
        cv::imshow("Original vs Undistorted", combined_image);
        cv::waitKey(0);
    }
    
    return true;
}