#include "ManualCalib.hpp"

bool calibrateCameraFromImages(const std::string &folder_path, cv::Size board_size, float square_size, cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
{
    std::vector<cv::String> image_paths;
    for (const auto &entry : fs::directory_iterator(folder_path))
    {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".bmp")
        {
            image_paths.push_back(entry.path().string());
        }
    }

    if (image_paths.empty())
    {
        std::cerr << "未找到任何图片文件！" << std::endl;
        return false;
    }

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    // 生成棋盘格的三维坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < board_size.height; ++i)
    {
        for (int j = 0; j < board_size.width; ++j)
        {
            obj.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    // 检测棋盘格角点
    for (const auto &path : image_paths)
    {
        cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);

        std::vector<cv::Point2f> corners;

        bool found = cv::findChessboardCorners(image, board_size, corners);
        if (found)
        {
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            image_points.push_back(corners);
            object_points.push_back(obj);
        }
    }

    if (image_points.empty())
    {
        std::cerr << "未检测到有效的棋盘格角点！" << std::endl;
        return false;
    }

    // 进行相机标定
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(object_points, image_points, cv::Size(image_points[0][0].x, image_points[0][0].y),
                                     camera_matrix, dist_coeffs, rvecs, tvecs);

    std::cout << "标定完成，重投影误差: " << rms << std::endl;
    std::cout << "相机内参矩阵:\n"
              << camera_matrix << std::endl;
    std::cout << "畸变系数:\n"
              << dist_coeffs << std::endl;

    // 保存内参矩阵和畸变系数到文件
    cv::FileStorage fs("E:/Source/4DRadarTest/camera_calibration.yml", cv::FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "camera_matrix" << camera_matrix;
        fs << "dist_coeffs" << dist_coeffs;
        fs.release();
        std::cout << "内参矩阵和畸变系数已保存到 camera_calibration.yml" << std::endl;
    }
    else
    {
        std::cerr << "无法保存内参矩阵和畸变系数！" << std::endl;
        return false;
    }

    // 验证图像
    for (const auto &path : image_paths)
    {
        cv::Mat image = cv::imread(path, cv::IMREAD_GRAYSCALE);
        // 获取原始图像的尺寸
        int original_width = image.cols;
        int original_height = image.rows;

        // 计算缩放后的尺寸
        int new_width = original_width / 4;   // 宽度缩小
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

//==============================================================================
// 外参标定相关函数
//==============================================================================
std::vector<cv::Mat> calib()
{
    std::vector<cv::Mat> calib_result;
    // 雷达点（3D 点）- 每组观测需要是vector<Point3f>
    // std::vector<cv::Point3f> radar_points_single = {
    //     {2.99, 16.44, 0.61},
    //     {-1.76, 29.50, 1.05},
    //     {17.19, 58.31, -0.38},
    //     {29.37, 95.45, 0.17},
    //     {-15.39, 103.64, 6.96},
    //     {-5.24, 37.73, -0.11},
    //     {-3.63, 13.93, -0.09},
    //     {-0.83, 16.18, 0.39},
    //     {-3.73, 23.09, -0.64}};
    std::vector<cv::Point3f> radar_points_single = {
        {2.99, 16.44, 0.61},
        {-1.76, 29.50, 1.05},
        {17.19, 58.31, -0.38},
        {29.37, 95.45, 0.17},
        {-5.24, 37.73, -0.11},
        {-3.63, 13.93, -0.09},
        {-0.83, 16.18, 0.39},
        {-3.73, 23.09, -0.64},
        {-12.83, 174.75, 12.77},
        {-11.44, 113.4, 8.55},
        {-7.94, 97.74, 7.36},
        {-0.32, 86.47, 7.55},
        {1.98, 80.87, 4.87},
        {-10.14, 52.06, 4.07},
        {-1.58, 42.5, 2.55},
        {4.90, 29.61, 2.33},
        {0.75, 26.42, 1.91},
        {-6.88, 15.05, 1.51}};

    std::vector<std::vector<cv::Point3f>> radar_points(1, radar_points_single);

    // std::vector<cv::Point2f> image_points_single = {
    //     {1467, 304},
    //     {979, 283},
    //     {1634, 331},
    //     {1655, 330},
    //     {753, 244},
    //     {834, 370},
    //     {606, 414},
    //     {990, 399},
    //     {786, 382}};
    std::vector<cv::Point2f> image_points_single = {
        {1467, 304},
        {979, 283},
        {1634, 331},
        {1655, 330},
        {834, 370},
        {606, 414},
        {990, 399},
        {786, 382},
        {958, 205},
        {904, 201},
        {943, 198},
        {1089, 203},
        {1147, 208},
        {732, 205},
        {1024, 212},
        {1413, 245},
        {1160, 229},
        {301, 250}};
    std::vector<std::vector<cv::Point2f>> image_points(1, image_points_single);

    int image_width = 1920;
    int image_height = 1080;
    // // 定义相机内参矩阵的初始值（使用更合理的初始估计）
    // cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    // camera_matrix.at<double>(0,0) = 1000.0; // fx
    // camera_matrix.at<double>(1,1) = 1000.0; // fy
    // camera_matrix.at<double>(0,2) = image_width/2.0;  // cx
    // camera_matrix.at<double>(1,2) = image_height/2.0; // cy

    // // 定义畸变系数
    // cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    cv::FileStorage fs("camera_calibration.yml", cv::FileStorage::READ);
    cv::Mat camera_matrix, dist_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["dist_coeffs"] >> dist_coeffs;
    fs.release();

    // 定义外参的旋转向量和平移向量
    // std::vector<cv::Mat> rvecs, tvecs;

    // 设置标定参数标志
    // int flags = cv::CALIB_USE_INTRINSIC_GUESS |
    //             cv::CALIB_FIX_PRINCIPAL_POINT;

    // 使用 calibrateCamera 进行标定
    // double reprojection_error = cv::calibrateCamera(
    //     radar_points,      // 3D 点
    //     image_points,      // 2D 点
    //     cv::Size(image_width, image_height), // 图像尺寸
    //     camera_matrix,     // 输出内参矩阵
    //     dist_coeffs,       // 输出畸变系数
    //     rvecs,            // 输出旋转向量
    //     tvecs,            // 输出平移向量
    //     flags             // 标定参数标志
    // );

    // 使用 solvePnP 求解外参
    cv::Mat rvec, tvec;
    // 初始化外参，便于使用SOLVEPNP_ITERATIVE
    // rvec = (cv::Mat_<double>(3, 1) << 1.57, 0, 0);
    // tvec = (cv::Mat_<double>(3, 1) << 0, 0, 3);
    bool success = cv::solvePnP(radar_points_single, image_points_single, camera_matrix, dist_coeffs, rvec, tvec, false, 1);

    if (!success || rvec.empty() || tvec.empty())
    {
        std::cerr << "错误：solvePnP 未能求解外参！" << std::endl;
        return calib_result;
    }

    calib_result.push_back(camera_matrix);
    calib_result.push_back(dist_coeffs);
    calib_result.push_back(rvec);
    calib_result.push_back(tvec);

    // 输出结果
    std::cout << "相机内参矩阵 K:\n"
              << camera_matrix << std::endl;
    std::cout << "畸变系数:\n"
              << dist_coeffs << std::endl;
    std::cout << "旋转向量 rvec:\n"
              << rvec << std::endl;
    std::cout << "平移向量 tvec:\n"
              << tvec << std::endl;
    // std::cout << "重投影误差: " << reprojection_error << std::endl;

    // 验证重投影误差
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(radar_points_single, rvec, tvec,
                      camera_matrix, dist_coeffs, projected_points);

    // 计算每个点的重投影误差
    for (size_t i = 0; i < projected_points.size(); i++)
    {
        double error = cv::norm(image_points_single[i] - projected_points[i]);
        std::cout << "Point " << i << " reprojection error: "
                  << error << " pixels" << std::endl;
    }

    // 保存外参到 YAML 文件
    cv::FileStorage fs_out("E:/Source/4DRadarTest/extrinsic_calibration.yml", cv::FileStorage::WRITE);
    if (fs_out.isOpened())
    {
        fs_out << "rotation_vector" << rvec;
        fs_out << "translation_vector" << tvec;
        fs_out.release();
        std::cout << "外参已保存到 extrinsic_calibration.yml" << std::endl;
    }
    else
    {
        std::cerr << "无法打开文件以保存外参！" << std::endl;
    }

    return calib_result;
}

void projectRadarPoints(const std::vector<RadarPoint> &radar_points,
                        cv::Mat &image,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &dist_coeffs,
                        const cv::Mat &rvec,
                        const cv::Mat &tvec)
{
    // 转换雷达点为OpenCV格式
    std::vector<cv::Point3f> object_points;
    bool mode = false; // true表示启用速度过滤
    for (const auto &point : radar_points)
    {
        if (mode == true && std::abs(point.v_r) <= 0.1)
        {
            continue; // 跳过不满足速度条件的点
        }
        object_points.push_back(cv::Point3f(point.x, point.y, point.z));
    }
    if (object_points.empty())
        return;

    // 投影3D点到图像平面
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    // 在图像上绘制投影点
    for (int i = 0; i < image_points.size(); i++)
    {
        // 检查点是否在图像范围内
        if (image_points[i].x >= 0 && image_points[i].x < image.cols &&
            image_points[i].y >= 0 && image_points[i].y < image.rows)
        {

            // 获取当前点的深度 y 值，并限制在 [0, 250] 范围内
            float y_depth = object_points[i].y;
            y_depth = std::max(0.0f, std::min(y_depth, 250.0f));

            // 根据深度计算颜色（HSV 色调从红到蓝）
            float hue = (y_depth / 250.0f) * 240.0f; // 0（红）~240（蓝）
            uchar h = static_cast<uchar>(hue / 2);   // OpenCV 的 H 范围为 [0, 180]
            cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, 255, 255));
            cv::Mat bgr;
            cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
            cv::Vec3b color_bgr = bgr.at<cv::Vec3b>(0, 0);
            cv::Scalar color(color_bgr[0], color_bgr[1], color_bgr[2]); // BGR 格式

            // 根据深度计算点的大小（近大远小）
            const float max_radius = 5.0f;
            const float min_radius = 1.0f;
            float radius = max_radius - (y_depth / 250.0f) * (max_radius - min_radius);
            int radius_int = static_cast<int>(std::round(radius));

            // 绘制点
            cv::circle(image, image_points[i], radius_int, color, -1);
        }
    }
}

CalibRadarVisualizer::CalibRadarVisualizer(
    const std::string &calib_file,
    const std::string &extrinsic_file,
    const std::string &radar_path,
    const std::string &image_path)
{
    // Load calibration parameters
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["dist_coeffs"] >> dist_coeffs_;
    fs.release();

    // Load extrinsic parameters
    fs.open(extrinsic_file, cv::FileStorage::READ);
    fs["rotation_vector"] >> rvec_;
    fs["translation_vector"] >> tvec_;
    source_rvec = rvec_.clone();
    source_tvec = tvec_.clone();
    fs.release();
    extrinsic_params_.rx = rvec_.at<double>(0);
    extrinsic_params_.ry = rvec_.at<double>(1);
    extrinsic_params_.rz = rvec_.at<double>(2);
    extrinsic_params_.tx = tvec_.at<double>(0);
    extrinsic_params_.ty = tvec_.at<double>(1);
    extrinsic_params_.tz = tvec_.at<double>(2);
    updateExtrinsicParams();

    // Initialize readers
    radar_reader_ = std::dynamic_pointer_cast<RadarFileReader>(DataReaderFactory::createReader(ReaderType::RADAR_FILE, radar_path));
    image_reader_ = std::dynamic_pointer_cast<ImageFileReader>(DataReaderFactory::createReader(ReaderType::IMAGE_FILE, image_path));

    if (!radar_reader_->init() || !image_reader_->init())
    {
        throw std::runtime_error("Failed to initialize readers");
    }

    mapper.init(calib_file, extrinsic_file);

    // Initialize visualizers
    initializeVisualizers();

    // Create control panel
    createControlPanel();
}

void CalibRadarVisualizer::resetExtrinsicParams()
{
    // extrinsic_params_ = ExtrinsicParams(); // 使用默认值重置
    extrinsic_params_.rx = source_rvec.at<double>(0);
    extrinsic_params_.ry = source_rvec.at<double>(1);
    extrinsic_params_.rz = source_rvec.at<double>(2);
    extrinsic_params_.tx = source_tvec.at<double>(0);
    extrinsic_params_.ty = source_tvec.at<double>(1);
    extrinsic_params_.tz = source_tvec.at<double>(2);
    updateExtrinsicParams();
    std::cout << "Extrinsic parameters reset to default" << std::endl;
}

void CalibRadarVisualizer::saveExtrinsicParams()
{
    cv::FileStorage fs("new_extrinsic_calibration.yml", cv::FileStorage::WRITE);
    fs << "rotation_vector" << rvec_;
    fs << "translation_vector" << tvec_;
    fs.release();
    std::cout << "Extrinsic parameters saved to file" << std::endl;
}

void CalibRadarVisualizer::keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.keyDown())
    {
        switch (event.getKeyCode())
        {
        case ' ': // 空格键控制暂停/继续
            pause_ = !pause_;
            std::cout << (pause_ ? "Paused" : "Playing") << std::endl;
            break;

        case 'n': // 'n'键显示下一帧
        case 'N':
            if (pause_)
            {
                processNextFrame();
            }
            break;

        case 'r': // 'r'键重置外参
        case 'R':
            resetExtrinsicParams();
            break;

        case 's': // 's'键保存当前外参
        case 'S':
            saveExtrinsicParams();
            break;

        case 'q': // 'q'键退出程序
        case 'Q':
        case 27: // ESC键
            viewer_->close();
            viewer_source_->close();
            break;
        }
    }
}

void CalibRadarVisualizer::initializeVisualizers()
{
    // Initialize display manager
    display_manager_.addVisualizer(DisplayManager::DisplayType::IMAGE,
                                   std::make_shared<ImageVisualizer>());
    display_manager_.setLayout(DisplayManager::DisplayType::IMAGE, 150, 400, 800, 600);

    // Initialize PCL visualizers
    viewer_.reset(new pcl::visualization::PCLVisualizer("DBSCAN with Bounding Boxes"));
    viewer_->setBackgroundColor(0, 0, 0);
    // viewer_->registerKeyboardCallback(&CalibRadarVisualizer::keyboardCallback, *this);

    viewer_source_.reset(new pcl::visualization::PCLVisualizer("Source cloud"));
    viewer_source_->setBackgroundColor(0, 0, 0);
    viewer_source_->registerKeyboardCallback(&CalibRadarVisualizer::keyboardCallback, *this);
    // 添加网格
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> gridLines;
    for (int i = 0; i < 11; ++i)
    {
        float x = -250.0f + i * 50.0f;
        float y = -250.0f + i * 50.0f;

        // 垂直线
        gridLines.push_back({pcl::PointXYZ(x, -250.0f, 0),
                             pcl::PointXYZ(x, 250.0f, 0)});

        // 水平线
        gridLines.push_back({pcl::PointXYZ(-250.0f, y, 0),
                             pcl::PointXYZ(250.0f, y, 0)});
    }
    // 添加网格
    for (size_t i = 0; i < gridLines.size(); ++i)
    {
        viewer_source_->addLine(
            gridLines[i].first,
            gridLines[i].second,
            0.5, 0.5, 0.5,
            "grid_line_" + std::to_string(i));
    }
}

void CalibRadarVisualizer::createControlPanel()
{
    // Create control window
    cv::namedWindow("Controls", cv::WINDOW_NORMAL);

    // constexpr int SCALE = 1000; // 用于提高精度的缩放因子

    // 旋转参数滑动条 (弧度制: -π 到 π)
    cv::createTrackbar("Rotation X", "Controls", nullptr, 2 * 1000, [](int val, void *userdata)
                       {
            auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
            viz->extrinsic_params_.rx = (val - 1000) * (M_PI/32) / 1000 + viz->source_rvec.at<double>(0);
            viz->updateExtrinsicParams();

            std::cout << "[DEBUG] Rotation X: " << viz->extrinsic_params_.rx << " rad" << std::endl; }, this);

    cv::createTrackbar("Rotation Y", "Controls", nullptr, 2 * 1000, [](int val, void *userdata)
                       {
            auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
            viz->extrinsic_params_.ry = (val - 1000) * (M_PI/32) / 1000 + viz->source_rvec.at<double>(1);
            viz->updateExtrinsicParams();

            std::cout << "[DEBUG] Rotation Y: " << viz->extrinsic_params_.ry << " rad" << std::endl; }, this);

    cv::createTrackbar("Rotation Z", "Controls", nullptr, 2 * 1000, [](int val, void *userdata)
                       {
            auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
            viz->extrinsic_params_.rz = (val - 1000) * (M_PI/32) / 1000 + viz->source_rvec.at<double>(2);
            viz->updateExtrinsicParams();

            std::cout << "[DEBUG] Rotation Z: " << viz->extrinsic_params_.rz << " rad" << std::endl; }, this);

    // 平移参数滑动条 (米: -5m 到 5m)
    cv::createTrackbar("Translation X", "Controls", nullptr, 2 * 1000, [](int val, void *userdata)
                       {
            auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
            viz->extrinsic_params_.tx = (val - 1000) * 7.0 / 1000 + viz->source_tvec.at<double>(0);
            viz->updateExtrinsicParams();

            std::cout << "[DEBUG] Translation X: " << viz->extrinsic_params_.tx << " m" << std::endl; }, this);

    cv::createTrackbar("Translation Y", "Controls", nullptr, 2 * 1000, [](int val, void *userdata)
                       {
            auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
            viz->extrinsic_params_.ty = (val - 1000) * 7.0 / 1000 + viz->source_tvec.at<double>(1);
            viz->updateExtrinsicParams();
            std::cout << "[DEBUG] Translation Y: " << viz->extrinsic_params_.ty << " m" << std::endl; }, this);

    cv::createTrackbar("Translation Z", "Controls", nullptr, 2 * 1000, [](int val, void *userdata)
                       {
            auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
            viz->extrinsic_params_.tz = (val - 1000) * 7.0 / 1000 + viz->source_tvec.at<double>(2);
            viz->updateExtrinsicParams();
            std::cout << "[DEBUG] Translation Z: " << viz->extrinsic_params_.tz << " m" << std::endl; }, this);

    // 添加"下一帧"按钮
    // cv::createButton("Next Frame", [](int state, void *userdata)
    //                  {
    //         auto* viz = static_cast<CalibRadarVisualizer*>(userdata);
    //         viz->processNextFrame(); }, this, cv::QT_PUSH_BUTTON);
}

void CalibRadarVisualizer::updateExtrinsicParams()
{
    // Update rotation vector
    cv::Mat rotation_matrix;
    cv::Rodrigues(cv::Vec3d(extrinsic_params_.rx,
                            extrinsic_params_.ry,
                            extrinsic_params_.rz),
                  rotation_matrix);
    cv::Rodrigues(rotation_matrix, rvec_);

    // Update translation vector
    tvec_ = (cv::Mat_<double>(3, 1) << extrinsic_params_.tx,
             extrinsic_params_.ty,
             extrinsic_params_.tz);
    
    mapper.setExtrinsicParam(rvec_, tvec_);

    params_changed_ = true;
}

void CalibRadarVisualizer::processCurrentFrame()
{
    auto radar_data = std::dynamic_pointer_cast<RadarData>(radar_reader_->getData());
    auto image_data = std::dynamic_pointer_cast<ImageData>(image_reader_->getData());

    // 检查数据是否有效
    if (!radar_data || !image_data)
    {
        std::cerr << "Error: Invalid data pointers!" << std::endl;
        return;
    }

    results = model.infer(image_data->frame);
    depthMask = mapper.createDepthMask(radar_data->points, image_data->frame.size());

    // Process and display radar data
    processRadarData(radar_data);

    // Process and display image data
    processImageData(image_data, radar_data);
}

void CalibRadarVisualizer::processNextFrame()
{
    if (radar_reader_->isEnd())
        return;

    radar_reader_->readNext();
    image_reader_->readNext();

    auto radar_data = std::dynamic_pointer_cast<RadarData>(radar_reader_->getData());
    auto image_data = std::dynamic_pointer_cast<ImageData>(image_reader_->getData());

    results = model.infer(image_data->frame);
    depthMask = mapper.createDepthMask(radar_data->points, image_data->frame.size());

    // Process and display radar data
    processRadarData(radar_data);

    // Process and display image data
    processImageData(image_data, radar_data);
}

void CalibRadarVisualizer::processRadarData(const std::shared_ptr<RadarData> &radar_data)
{
    // Clear previous visualizations
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
    viewer_source_->removeAllPointClouds();
    // viewer_source_->removeAllShapes();

    const float MAX_ABS_SPEED = 20.0f; // 预设最大速度绝对值

    // Convert radar points to PCL format
    // ?auto [input_cloud, source_cloud] = convertRadarToPCL(radar_data);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (const auto &point : radar_data->points)
    {
        pcl::PointXYZI p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        input_cloud->push_back(p);

        pcl::PointXYZRGB p_rcs;
        p_rcs.x = point.x;
        p_rcs.y = point.y;
        p_rcs.z = point.z;
        // 计算颜色（HSV -> RGB）
        float speed = point.v_r;
        float abs_speed = std::min(std::abs(speed), MAX_ABS_SPEED);
        float hue = 0.0;

        // 颜色映射规则
        if (speed < 0)
        {
            // 正速度：红(0°) -> 黄(60°)
            hue = 60.0f - 60.0f * (abs_speed / MAX_ABS_SPEED);
        }
        else if (speed > 0)
        {
            // 负速度：蓝(240°) -> 青(180°)
            hue = 180.0f + 60.0f * (abs_speed / MAX_ABS_SPEED);
        }
        else
        {
            hue = 120.0f;
        }

        // 转换HSV到RGB
        cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(hue, 1.0, 1.0));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

        // 设置点云颜色
        p_rcs.r = static_cast<uint8_t>(bgr.at<cv::Vec3f>(0, 0)[2] * 255);
        p_rcs.g = static_cast<uint8_t>(bgr.at<cv::Vec3f>(0, 0)[1] * 255);
        p_rcs.b = static_cast<uint8_t>(bgr.at<cv::Vec3f>(0, 0)[0] * 255);
        source_cloud->push_back(p_rcs);
    }

    // Perform DBSCAN clustering
    // int num_clusters = PCLTools::dbscan(input_cloud, eps_, min_pts_);

    // Visualize clusters and bounding boxes
    // visualizeClusters(input_cloud, num_clusters);

    // Visualize source cloud
    // ?visualizeSourceCloud(source_cloud);
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(source_cloud, "intensity");
    // viewer_source_->addPointCloud<pcl::PointXYZI>(source_cloud, intensity_distribution, "source cloud");
    viewer_source_->addPointCloud<pcl::PointXYZRGB>(source_cloud, "source cloud");

    // 绘制框
    mapper.visualizeDetections(viewer_source_, results, depthMask);
}

void CalibRadarVisualizer::processImageData(const std::shared_ptr<ImageData> &image_data,
                                            const std::shared_ptr<RadarData> &radar_data)
{
    cv::Mat display_image = image_data->frame.clone();

    // Project radar points onto image
    projectRadarPoints(radar_data->points,
                       display_image,
                       camera_matrix_,
                       dist_coeffs_,
                       rvec_,
                       tvec_);

    for (const auto& res : results) {
        cv::rectangle(display_image, res.box, cv::Scalar(0, 255, 0), 2);
        std::string label = std::to_string(res.class_id) + ": " + std::to_string(res.confidence);
        cv::putText(display_image, label, res.box.tl(), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    }
    // Update display
    auto display_imagedata = std::make_shared<ImageData>();
    display_imagedata->frame = display_image;
    display_manager_.updateDisplay(display_imagedata);
    display_manager_.renderAll();
}

void CalibRadarVisualizer::run()
{
    if (radar_reader_->isEnd())
        return;

    radar_reader_->readNext();
    image_reader_->readNext();

    while (!radar_reader_->isEnd())
    {
        if (!pause_ || params_changed_)
        {
            // radar_reader_->readNext();
            // image_reader_->readNext();
            // processNextFrame();
            processCurrentFrame();
            params_changed_ = false;
        }

        viewer_->spinOnce(10);
        viewer_source_->spinOnce(10);

        // Handle GUI events
        cv::waitKey(1);
    }
}

//==============================================================================
// 通用映射类
//==============================================================================
bool RadarImageMapper::isValidPoint(const RadarPoint &pt)
{
    // 有效性检查（示例：速度过滤+深度范围）
    return std::abs(pt.v_r) > 0.0f &&    // 有效速度
           pt.y > 0.0f && pt.y <= 300.0f; // 深度在0-300米
}

bool RadarImageMapper::init(const std::string &calib_file, const std::string &extrinsic_file)
{
    cv::FileStorage fs(calib_file, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camera_matrix_;
    fs["dist_coeffs"] >> dist_coeffs_;
    fs.release();

    fs.open(extrinsic_file, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["rotation_vector"] >> rvec_;
    fs["translation_vector"] >> tvec_;
    fs.release();

    return true;
}

void RadarImageMapper::setExtrinsicParam(cv::Mat rvec, cv::Mat tvec){
    rvec_ = rvec;
    tvec_ = tvec;
}

cv::Mat RadarImageMapper::createDepthMask(const std::vector<RadarPoint> &radar_points,
                                          const cv::Size &image_size,
                                          float max_depth,
                                          bool use_nearest)
{
    cv::Mat depth_mask = cv::Mat::zeros(image_size, CV_32FC1);
    std::vector<cv::Point3f> object_points;
    std::vector<size_t> valid_indices;

    // 转换有效雷达点到3D坐标（自动过滤无效点）
    for (size_t i = 0; i < radar_points.size(); ++i)
    {
        if (isValidPoint(radar_points[i]) && radar_points[i].y <= max_depth)
        {
            object_points.emplace_back(radar_points[i].x, radar_points[i].y, radar_points[i].z);
            valid_indices.push_back(i);
        }
    }

    if (object_points.empty())
        return depth_mask;

    // 投影到图像平面
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rvec_, tvec_,
                      camera_matrix_, dist_coeffs_, image_points);

    // 生成深度掩膜（保持原始深度值）
    for (size_t i = 0; i < image_points.size(); ++i)
    {
        const auto &img_pt = image_points[i];
        if (img_pt.x >= 0 && img_pt.x < image_size.width &&
            img_pt.y >= 0 && img_pt.y < image_size.height)
        {
            const float depth = object_points[i].y; // 原始深度值
            const int x = static_cast<int>(img_pt.x);
            const int y = static_cast<int>(img_pt.y);

            if (use_nearest)
            {
                // 保留最近深度
                float &current = depth_mask.at<float>(y, x);
                if (current == 0 || depth < current)
                {
                    current = depth;
                }
            }
            else
            {
                // 累加深度求平均
                depth_mask.at<float>(y, x) += depth;
            }
        }
    }

    // 后处理（仅非最近邻模式需要）
    if (!use_nearest)
    {
        cv::Mat count_mat = cv::Mat::zeros(image_size, CV_32FC1);
        for (const auto &img_pt : image_points)
        {
            if (img_pt.x >= 0 && img_pt.x < image_size.width &&
                img_pt.y >= 0 && img_pt.y < image_size.height)
            {
                count_mat.at<float>(img_pt.y, img_pt.x) += 1.0f;
            }
        }
        cv::divide(depth_mask, count_mat, depth_mask);
    }

    // 应用深度上限（300米）
    cv::threshold(depth_mask, depth_mask, max_depth, max_depth, cv::THRESH_TRUNC);

    return depth_mask;
}

float RadarImageMapper::calculateBoxDepth(const cv::Rect &box, const cv::Mat &depth_mask)
{
    // 获取有效ROI区域
    cv::Rect valid_roi = box & cv::Rect(0, 0, depth_mask.cols, depth_mask.rows);
    if (valid_roi.area() <= 0)
        return 0.0f;

    // 提取ROI区域
    cv::Mat roi = depth_mask(valid_roi);

    // 创建有效点掩膜（深度>0.1）
    cv::Mat mask = (roi > 0.1f);

    // 计算非零区域均值（使用cv::mean）
    cv::Scalar mean_val = cv::mean(roi, mask);

    // 有效性验证（至少3个有效点）
    int valid_pixels = cv::countNonZero(mask);
    return (valid_pixels >= 3) ? static_cast<float>(mean_val[0]) : 0.0f;
}

cv::Vec3f RadarImageMapper::estimateObjectSize(const cv::Rect &box, float depth, int class_id)
{
    // 已知典型物体高度（单位：米）
    const std::map<int, float> CLASS_HEIGHTS = {
        {0, 1.7f}, // 行人高度
        {1, 1.5f}  // 车辆高度
    };

    // 获取类别默认高度
    float height = 2.0f; // 默认高度
    if (CLASS_HEIGHTS.find(class_id) != CLASS_HEIGHTS.end())
    {
        height = CLASS_HEIGHTS.at(class_id);
    }

    // 根据像素高度估算实际宽度（透视投影模型）
    if (depth > 0.1f && box.height > 0)
    {
        const float pixel_height = box.height;
        const float actual_width = (box.width / pixel_height) * height;
        return cv::Vec3f(actual_width, actual_width * 0.5f, height); // 长、高、宽
    }

    return cv::Vec3f(1.0f, 1.0f, 1.0f); // 默认尺寸
}

void RadarImageMapper::addBoundingBoxToViewer(pcl::visualization::PCLVisualizer::Ptr viewer,
                                              const cv::Point3f &center,
                                              const cv::Vec3f &size,
                                              const std::string &id)
{
    // 计算边界坐标
    const float x_min = center.x - size[0] / 2;
    const float x_max = center.x + size[0] / 2;
    const float y_min = center.y - size[1] / 2;
    const float y_max = center.y + size[1] / 2;
    const float z_min = center.z - size[2] / 2;
    const float z_max = center.z + size[2] / 2;

    // 调用正确重载函数
    viewer->addCube(
        x_min, x_max,
        y_min, y_max,
        z_min, z_max,
        1.0, 1.0, 0.0, // 绿色
        id);
}

void RadarImageMapper::visualizeDetections(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<InferenceModel::DetectionResult> &results, const cv::Mat &depth_mask)
{
    // 清空之前的可视化元素
    viewer->removeAllShapes();

    // 处理每个检测结果
    for (const auto &res : results)
    {
        // 计算中心点深度
        float depth = calculateBoxDepth(res.box, depth_mask);
        std::cout << "depth: " << depth << std::endl;

        if (depth > 0.1f && depth <= 300.0f)
        {
            // 转换为3D坐标
            cv::Point2f center_px(
                res.box.x + res.box.width / 2.0f,
                res.box.y + res.box.height / 2.0f);
            cv::Point3f world_pt = imageToRadar(center_px, depth);

            // 估算物体尺寸
            auto size = estimateObjectSize(res.box, depth, res.class_id);

            // 生成唯一ID
            std::string id = "obj_" + std::to_string(res.class_id) + "_" + std::to_string(&res - &results[0]);

            // 添加3D边界框
            addBoundingBoxToViewer(viewer, world_pt, size, id);
        }
    }
}

cv::Mat RadarImageMapper::mapperProject(const std::vector<RadarPoint> &radar_points,
                                        cv::Mat &image)
{
    projectRadarPoints(radar_points,
        image,
        camera_matrix_,
        dist_coeffs_,
        rvec_,
        tvec_);
    return image;
}

cv::Point3f RadarImageMapper::imageToRadar(const cv::Point2f &image_point, float depth)
{
    // // Step 1: 畸变校正
    // std::vector<cv::Point2f> distorted_points{image_point};
    // std::vector<cv::Point2f> undistorted_points;

    // cv::undistortPoints(
    //     distorted_points,
    //     undistorted_points,
    //     camera_matrix_,
    //     dist_coeffs_);

    // cv::Point2f corrected_point = undistorted_points[0];

    cv::Mat uv = (cv::Mat_<double>(3, 1) << image_point.x, image_point.y, 1.0);
    cv::Mat K_inv = camera_matrix_.inv();
    cv::Mat camera_point = K_inv * uv * depth;

    cv::Mat R;
    cv::Rodrigues(rvec_, R);
    cv::Mat radar_point = R.t() * (camera_point - tvec_);

    std::cout << "x=" << static_cast<float>(radar_point.at<double>(0)) << std::endl;
    std::cout << "y=" << static_cast<float>(radar_point.at<double>(1)) << std::endl;
    std::cout << "z=" << static_cast<float>(radar_point.at<double>(2)) << std::endl;

    return cv::Point3f(
        static_cast<float>(radar_point.at<double>(0)),
        static_cast<float>(radar_point.at<double>(1)),
        static_cast<float>(radar_point.at<double>(2)));
}
