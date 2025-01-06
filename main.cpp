#include "TcpCommandHandler.hpp"  // 包含了所有Windows相关定义
#include "SensorData.hpp"
#include "DataReaderFactory.hpp"
#include "DisplayManager.hpp"   // 其中既有eigen又有opencv
#include "SynchronizedCollector.hpp"
#include "PclTools.hpp"
// #include "toolstest.hpp"
#include <filesystem>
#include <opencv2/core/utils/logger.hpp>
#include <pcl/visualization/cloud_viewer.h>


// 用于打印十六进制数据
void printHex(const std::vector<uint8_t>& data) {
    for (uint8_t byte : data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

// void testVideoReader() {
//     // 本地视频文件路径
//     std::string videoPath = "E:/dataset/How Tower Cranes Build Themselves1080p.mp4";  // 替换为你的视频路径
//     std::string savePath = "test_output.avi";  // 保存路径
    
//     // 使用工厂类创建视频读取器
//     auto reader = DataReaderFactory::createReader(ReaderType::VIDEO_STREAM, videoPath);
//     if (!reader) {
//         std::cerr << "Failed to create video reader!" << std::endl;
//         return;
//     }
    
//     // 转换为VideoStreamReader以使用特定功能
//     auto videoReader = std::dynamic_pointer_cast<VideoStreamReader>(reader);
//     if (!videoReader) {
//         std::cerr << "Failed to cast to VideoStreamReader!" << std::endl;
//         return;
//     }
    
//     // 初始化
//     if (!videoReader->init()) {
//         std::cerr << "Failed to initialize video reader!" << std::endl;
//         return;
//     }
    
//     // 启用保存
//     if (!videoReader->enableSave(savePath, 30.0)) {
//         std::cerr << "Failed to enable video saving!" << std::endl;
//         return;
//     }
    
//     int frameCount = 0;
//     // 读取并保存视频帧
//     while (videoReader->grabFrame()) {
//         auto data = videoReader->getData();
//         if (data) {
//             frameCount++;
//             std::cout << "Frame " << frameCount 
//                       << ", Timestamp: " << data->timestamp << "ms" << std::endl;
            
//             // 显示当前帧
//             cv::Mat& frame = std::dynamic_pointer_cast<ImageData>(data)->frame;
//             cv::imshow("Video", frame);
//             if (cv::waitKey(1) == 27) {  // ESC键退出
//                 break;
//             }
//         }
//     }
    
//     videoReader->disableSave();
//     cv::destroyAllWindows();
//     std::cout << "Processed " << frameCount << " frames" << std::endl;
// }

// void testTcpClient() {
//     // 创建TCP客户端实例
//     TcpClient client("192.168.10.117", 50000);
    
//     // 尝试连接服务器
//     if (!client.connect()) {
//         std::cout << "连接服务器失败！" << std::endl;
//         return;
//     }
    
//     std::cout << "成功连接到服务器！" << std::endl;
    
//     // 准备发送的测试数据
//     std::vector<uint8_t> sendData = {0x01, 0x02, 0x03, 0x04, 0x05};
    
//     // 发送数据
//     ssize_t sendResult = client.write(sendData.data(), sendData.size());
//     if (sendResult < 0) {
//         std::cout << "发送数据失败！" << std::endl;
//         client.disconnect();
//         return;
//     }
    
//     std::cout << "成功发送 " << sendResult << " 字节的数据" << std::endl;
    
//     // 准备接收缓冲区
//     std::vector<uint8_t> recvBuffer(1024);
    
//     // 接收数
//     ssize_t recvResult = client.read(recvBuffer.data(), recvBuffer.size());
//     if (recvResult < 0) {
//         std::cout << "接收数据失败！" << std::endl;
//     } else if (recvResult > 0) {
//         std::cout << "接收到 " << recvResult << " 字节的数据：";
//         // 打印接收到的数据（十六进制格式）
//         for (ssize_t i = 0; i < recvResult; ++i) {
//             printf("%02X ", recvBuffer[i]);
//         }
//         std::cout << std::endl;
//     }
    
//     // 等待一段时间
//     std::this_thread::sleep_for(std::chrono::seconds(1));
    
//     // 断开连接
//     client.disconnect();
//     std::cout << "已断开连接" << std::endl;
// }

// 测试雷达 
void testRadar() {
    try {
        // 创建命令处理器
        TcpCommandHandler handler("192.168.53.168", 12345);
        
        // 创建保存目录
        std::string saveDir = "radar_data";
        if (!std::filesystem::exists(saveDir)) {
            std::filesystem::create_directory(saveDir);
        }
        
        // 连接雷达
        if (!handler.connect()) {
            std::cerr << "无法连接到雷达设备" << std::endl;
            return;
        }
        std::cout << "成功连接到雷达设备" << std::endl;

        // 主循环
        int count = 0;
        const int totalFrames = 200;
        const int printInterval = 10;  // 每10帧打印一次
        auto lastTime = std::chrono::steady_clock::now();
        auto lastPrintTime = std::chrono::steady_clock::now();
        
        while (count < totalFrames) {
            CommandParseResult result;
            if (handler.receiveAndParseFrame(result)) {
                if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
                    // 生成时间戳文件名
                    auto now = std::chrono::system_clock::now();
                    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
                    std::string filename = saveDir + "/radar_" + std::to_string(timestamp) + ".csv";
                    
                    // 保存当前帧数据
                    if (handler.startRecording(filename, "csv")) {
                        std::cout << "保存帧数据到: " << filename << std::endl;
                        std::cout << "接收到目标数量: " << targets->size() << std::endl;
                        handler.saveTargetData(*targets, "csv");
                        handler.stopRecording();
                    }
                    
                    // 每隔一定帧数才打印进度
                    if (count % printInterval == 0) {
                        auto now = std::chrono::steady_clock::now();
                        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - lastPrintTime).count();
                        lastPrintTime = now;
                        
                        float progress = (count + 1) * 100.0f / totalFrames;
                        std::cout << "\r采集进度: " << std::fixed << std::setprecision(1) 
                                  << progress << "% (" << (count + 1) << "/" << totalFrames 
                                  << "), 平均采集间隔: " << interval / printInterval << "ms" 
                                  << std::flush;  // 使用\r和std::flush实现原地更新
                    }
                }
                count++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << std::endl;  // 完成后换行
        
        handler.disconnect();
        std::cout << "测试完成，已断开连接" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "雷达测试异常: " << e.what() << std::endl;
    }
}

// void testMultiSourceCapture() {
//     try {
//         SynchronizedCollector collector;
//         std::cout << "创建采集器成功" << std::endl;
        
//         // 添加主数据源（雷达）
//         auto radarSource = std::make_unique<RadarSource>("192.168.10.117", 50000, "radar");
//         if (!radarSource->init()) {
//             throw std::runtime_error("雷达初始化失败");
//         }
//         collector.addSource(std::move(radarSource), true);
//         std::cout << "添加雷达源成功" << std::endl;
        
//         // 添加从数据源（视频）
//         // auto videoSource = std::make_unique<VideoSource>(
//         //     "E:/dataset/How Tower Cranes Build Themselves1080p.mp4", "camera1");
//         // // auto videoSource = std::make_unique<VideoSource>(
//         // //     "rtsp://192.168.88.216/live/chn0/stream_0", "camera1");
//         // if (!videoSource->init()) {
//         //     throw std::runtime_error("视频源初始化失败");
//         // }
//         // collector.addSource(std::move(videoSource), false);
//         // std::cout << "添加视频源成功" << std::endl;
        
//         collector.start();
        
//         while (true) {
//             if (cv::waitKey(1000) == 27) {
//                 std::cout << "检测到ESC键，准备退出..." << std::endl;
//                 break;
//             }
//             collector.printStats();
//         }
        
//         collector.stop();
//         std::cout << "采集已停止" << std::endl;
        
//     } catch (const std::exception& e) {
//         std::cerr << "多源采集异常: " << e.what() << std::endl;
//     }
// }

void testRadarFileReader() {
    std::cout << "开始测试雷达数据读取..." << std::endl;
    
    // 创建雷达数据读取器
    auto reader = DataReaderFactory::createReader(
        ReaderType::RADAR_FILE,
        "E:/dataset/radar_data_20241211/sync_data_20241211_151805/camera"  // 雷达数据文件夹路径
    );
    
    if (!reader) {
        std::cerr << "创建读取器失败" << std::endl;
        return;
    }
    
    // 初始化读取器
    if (!reader->init()) {
        std::cerr << "初始化读取器失败" << std::endl;
        return;
    }
    
    // 读取并打印数据
    int frameCount = 0;
    while (!reader->isEnd() && frameCount < 100) {  // 读取10帧数据
        if (reader->readNext()) {
            auto data = std::dynamic_pointer_cast<RadarData>(reader->getData());
            if (data) {
                std::cout << "帧 " << frameCount 
                         << " 时间戳: " << data->timestamp
                         << " 点数: " << data->points.size() << std::endl;
                
                // 打印第一个点的数据（如果有）
                if (!data->points.empty()) {
                    const auto& p = data->points[0];
                    std::cout << "第一个点: x=" << p.x 
                             << ", y=" << p.y 
                             << ", z=" << p.z 
                             << ", rcs=" << p.rcs 
                             << ", v_r=" << p.v_r << std::endl;
                }
            }
            frameCount++;
        }
    }
    
    std::cout << "测试完成，共读取 " << frameCount << " 帧数据" << std::endl;
}

void testDisplayManager(){
    try
    {
        std::cout << "开始测试显示管理器..." << std::endl;
        DisplayManager displayManager;

        // 添加可视化器
        displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
        displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());
        
        // 设置窗口布局
        displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);

        displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 150, 400, 800, 600);

        // 从文件中读取数据
        auto radar_reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/radar");
        if (!radar_reader->init()) {
            std::cerr << "初始化读取器失败" << std::endl;
            return;
        }

        auto image_reader = DataReaderFactory::createReader(ReaderType::IMAGE_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/camera_near");
        if (!image_reader->init()) {
            std::cerr << "初始化读取器失败" << std::endl;
            return;
        }
        
        // 读取数据
        image_reader->readNext();
        
        while (!radar_reader->isEnd()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(60));  // 改为5ms
            if (radar_reader->readNext()) {

                displayManager.updateDisplay(radar_reader->getData());
                
                while(std::dynamic_pointer_cast<ImageFileReader>(image_reader)->getData()->timestamp < std::dynamic_pointer_cast<RadarFileReader>(radar_reader)->getData()->timestamp){
                    image_reader->readNext();
                }

                displayManager.updateDisplay(image_reader->getData()); 
                displayManager.renderAll();
            }
        }
        
        // image_reader->readNext();
        // radar_reader->readNext();
        // while(true){
        //     std::this_thread::sleep_for(std::chrono::milliseconds(5));
        //     displayManager.updateDisplay(radar_reader->getData());
        //     displayManager.updateDisplay(image_reader->getData());
        //     displayManager.renderAll();
        // }
    }
    catch(const std::exception& e)
    {
        std::cerr << "testDisplayManager 异常: " << e.what() << '\n';
    }
}


void testDisplaySingleRadarPoint(){
    std::cout << "开始测试显示管理器..." << std::endl;
    DisplayManager displayManager;

    // 添加可视化器
    displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
    // 设置窗口布局
    displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);
}


void testRealTimeRadarDisplay(){
    try{
        SynchronizedCollector collector;
        std::cout << "创建采集器成功" << std::endl;
        
        // 添加主数据源（雷达）
        auto radarSource = std::make_unique<RadarSource>("192.168.88.219", 12345, "radar");
        if (!radarSource->init()) {
            throw std::runtime_error("雷达初始化失败");
        }
        collector.addSource(std::move(radarSource), true);
        std::cout << "添加雷达源成功" << std::endl;

        // 添加从视频源
        // auto videoSource = std::make_unique<VideoSource>(
        //     "E:/dataset/How Tower Cranes Build Themselves1080p.mp4", "camera");
        // auto videoSource = std::make_unique<VideoSource>(
        //     "rtsp://127.0.0.1:8554/camera_test", "camera");
        
        auto videoSource_near = std::make_unique<VideoSource>("rtsp://192.168.88.219:554/live/chn1/stream_1", "camera_near");
        if (!videoSource_near->init()) {
            throw std::runtime_error("视频源1初始化失败");
        }
        collector.addSource(std::move(videoSource_near), false);
        
        // auto videoSource_far = std::make_unique<VideoSource>("rtsp://192.168.88.219:554/live/chn0/stream_1", "camera_far");
        // if (!videoSource_far->init()) {
        //     throw std::runtime_error("视频源2初始化失败");
        // }
        // collector.addSource(std::move(videoSource_far), false);
        std::cout << "添加视频源成功" << std::endl;

        // 设置保存，(雷达，相机)
        collector.setSaveConfig(true, true, RadarFileReader::Format::BIN);

        // 创建可视化器
        DisplayManager displayManager;
        displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
        displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());

        // 设置窗口布局
        displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);
        displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 200, 150, 800, 600);
        // 启动线程
        collector.start();

        // 采集状态信息打印间隔
        int printInterval = 10;     // 按照雷达的采集频率
        int count = 0;

        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 改为5ms
            if(count % printInterval == 0){
                collector.printStats();
            }
            count++;
            auto newRadarData = collector.getMainSourceData();
            displayManager.updateDisplay(std::move(newRadarData));
            // 获取从源数据 
            auto subData = collector.getSubSourceData();
            for(const auto& data : subData){
                auto imageData = std::make_shared<ImageData>();
                imageData->frame = data.second;  // 正确设置 frame
                displayManager.updateDisplay(imageData);
            }
            // 渲染所有可视化器
            displayManager.renderAll();
        }

        collector.stop();
    }
    catch(const std::exception& e){
        std::cerr << "testDisplayManager 异常: " << e.what() << '\n';
    }
}

struct BoundingBox {
    pcl::PointXYZL min_pt;
    pcl::PointXYZL max_pt;
};
std::map<std::string, BoundingBox> boundingBoxes; // 存储立方体的最小点和最大点


void pointPickingEventOccurred(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
    pcl::visualization::PCLVisualizer::Ptr viewer = *static_cast<pcl::visualization::PCLVisualizer::Ptr*>(viewer_void);
    if(!event.getPointIndex() != -1){
        float x, y, z;
        event.getPoint(x, y, z);
        std::cout << "点坐标: (" << x << ", " << y << ", " << z << ")" << std::endl;

        // 查找包含点的立方体
        for(const auto& [box_id, bbox] : boundingBoxes){
            if (x >= bbox.min_pt.x && x <= bbox.max_pt.x &&
                y >= bbox.min_pt.y && y <= bbox.max_pt.y &&
                z >= bbox.min_pt.z && z <= bbox.max_pt.z)
            {
                std::cout << "立方体 ID: " << box_id << std::endl;
                std::cout << "长: " << bbox.max_pt.x - bbox.min_pt.x << std::endl;
                std::cout << "宽: " << bbox.max_pt.y - bbox.min_pt.y << std::endl;
                std::cout << "高: " << bbox.max_pt.z - bbox.min_pt.z << std::endl;
                std::cout << "质心: (" << (bbox.min_pt.x + bbox.max_pt.x) / 2 << ", "
                          << (bbox.min_pt.y + bbox.max_pt.y) / 2 << ", "
                          << (bbox.min_pt.z + bbox.max_pt.z) / 2 << ")" << std::endl;
                // 将选中的立方体颜色更改为黄色
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, box_id); // 黄色 (R=1.0, G=1.0, B=0.0)
                viewer->spinOnce(); // 刷新可视化窗口
                break;
            }
        }
    }
}

std::vector<cv::Mat> calib() {
    std::vector<cv::Mat> calib_result;
    // 雷达点（3D 点）- 每组观测需要是vector<Point3f>     
    std::vector<cv::Point3f> radar_points_single = {
        {-5.24, 37.73, -0.11},
        {-3.34, 29.92, -0.90},
        {-2.53, 19.24, -0.96},
        {-4.74, 32.34, -0.48},
        {-3.63, 13.93, -0.09},
        {-0.83, 16.18, 0.39},
        {-3.73, 23.09, -0.64}
    };
    std::vector<std::vector<cv::Point3f>> radar_points(1, radar_points_single);

    // 对应的图像像素点（2D 点）- 每组观测需要是vector<Point2f>
    std::vector<cv::Point2f> image_points_single = {
        {834, 370},
        {889, 370},
        {836, 381},
        {824, 370},
        {606, 414},
        {990, 399},
        {786, 382}
    };
    std::vector<std::vector<cv::Point2f>> image_points(1, image_points_single);

    int image_width = 1920;
    int image_height = 1080;
    // 定义相机内参矩阵的初始值（使用更合理的初始估计）
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0,0) = 1000.0; // fx
    camera_matrix.at<double>(1,1) = 1000.0; // fy
    camera_matrix.at<double>(0,2) = image_width/2.0;  // cx
    camera_matrix.at<double>(1,2) = image_height/2.0; // cy

    // 定义畸变系数
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    // 定义外参的旋转向量和平移向量
    std::vector<cv::Mat> rvecs, tvecs;


    // 设置标定参数标志
    int flags = cv::CALIB_USE_INTRINSIC_GUESS | 
                cv::CALIB_FIX_PRINCIPAL_POINT;

    // 使用 calibrateCamera 进行标定
    double reprojection_error = cv::calibrateCamera(
        radar_points,      // 3D 点
        image_points,      // 2D 点
        cv::Size(image_width, image_height), // 图像尺寸
        camera_matrix,     // 输出内参矩阵
        dist_coeffs,       // 输出畸变系数
        rvecs,            // 输出旋转向量
        tvecs,            // 输出平移向量
        flags             // 标定参数标志
    );

    calib_result.push_back(camera_matrix);
    calib_result.push_back(dist_coeffs);
    calib_result.push_back(rvecs[0]);
    calib_result.push_back(tvecs[0]);

    // 输出结果
    std::cout << "相机内参矩阵 K:\n" << camera_matrix << std::endl;
    std::cout << "畸变系数:\n" << dist_coeffs << std::endl;
    std::cout << "旋转向量 rvec:\n" << rvecs[0] << std::endl;
    std::cout << "平移向量 tvec:\n" << tvecs[0] << std::endl;
    std::cout << "重投影误差: " << reprojection_error << std::endl;

    // 验证重投影误差
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(radar_points_single, rvecs[0], tvecs[0], 
                     camera_matrix, dist_coeffs, projected_points);
    
    // 计算每个点的重投影误差
    for(size_t i = 0; i < projected_points.size(); i++) {
        double error = cv::norm(image_points_single[i] - projected_points[i]);
        std::cout << "Point " << i << " reprojection error: " 
                  << error << " pixels" << std::endl;
    }

    return calib_result;
}

void projectRadarPoints(const std::vector<RadarPoint>& radar_points, 
                       cv::Mat& image,
                       const cv::Mat& camera_matrix,
                       const cv::Mat& dist_coeffs,
                       const cv::Mat& rvec,
                       const cv::Mat& tvec) {
    // 转换雷达点为OpenCV格式
    std::vector<cv::Point3f> object_points;
    std::vector<float> object_points_speed;
    for (const auto& point : radar_points) {
        object_points.push_back(cv::Point3f(point.x, point.y, point.z));
        object_points_speed.push_back(point.v_r);
    }

    // 投影3D点到图像平面
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    // 在图像上绘制投影点
    for (int i = 0; i < image_points.size(); i++) {
        // 检查点是否在图像范围内
        if (image_points[i].x >= 0 && image_points[i].x < image.cols && 
            image_points[i].y >= 0 && image_points[i].y < image.rows) {
            // 绘制圆点，可以根据需要调整大小和颜色
            if(object_points_speed[i] < 0){
                cv::circle(image, image_points[i], 3, cv::Scalar(0, 0, 255), -1);
            }
            else if(object_points_speed[i] > 0){
                cv::circle(image, image_points[i], 3, cv::Scalar(255, 0, 0), -1);
            }
            // cv::circle(image, image_points[i], 3, cv::Scalar(0, 255, 0), -1);
        }
    }
}

void testDBScan(){
    std::vector<cv::Mat> calib_result = calib();
    cv::Mat camera_matrix = calib_result[0];
    cv::Mat dist_coeffs = calib_result[1];
    cv::Mat rvec = calib_result[2];
    cv::Mat tvec = calib_result[3];

    // 从文件中读取数据
    auto radar_reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/radar");
    if (!radar_reader->init()) {
        std::cerr << "初始化读取器失败" << std::endl;
        return;
    }

    auto image_reader = DataReaderFactory::createReader(ReaderType::IMAGE_FILE, "E:/Source/4DRadarTest/build/Debug/sync_data_20241231_155245/camera_near");
    if (!image_reader->init()) {
        std::cerr << "初始化读取器失败" << std::endl;
        return;
    }
    DisplayManager displayManager;
    // 添加可视化器
    displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());
    displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 150, 400, 800, 600);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("DBSCAN with Bounding Boxes"));
    viewer->setBackgroundColor(0, 0, 0);
    // 注册鼠标选点回调函数
    viewer->registerPointPickingCallback(pointPickingEventOccurred, &viewer);

    pcl::visualization::PCLVisualizer::Ptr viewer_source(new pcl::visualization::PCLVisualizer("Source cloud"));
    viewer_source->setBackgroundColor(0, 0, 0);

    // int index = 420;
    // for(int i = 0; i < index; i++){
    //     radar_reader->readNext();
    //     image_reader->readNext();
    // }

    while (!radar_reader->isEnd()) {
        // 每次更新前清空所有内容
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer_source->removeAllPointClouds();
        viewer_source->removeAllShapes();

        radar_reader->readNext();
        image_reader->readNext();

        auto currentRadarData = std::dynamic_pointer_cast<RadarData>(radar_reader->getData());
        // 聚类点云显示
        pcl::PointCloud<pcl::PointXYZL>::Ptr input_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
        pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for(const auto& point : currentRadarData->points){
            pcl::PointXYZI p_rcs;
            pcl::PointXYZL p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            input_cloud->push_back(p);
            p_rcs.x = point.x;
            p_rcs.y = point.y;
            p_rcs.z = point.z;
            p_rcs.intensity = point.v_r;
            source_cloud->push_back(p_rcs);
        }

        //DBSCAN参数
        float eps = 0.5;//dbscan半径参数
        int min_pts = 5;//dbscan聚类最小点数参数
        int number_of_clusters = 0;//计数器不需要修改

        // 使用pcl工具运行dbscan
        number_of_clusters = PCLTools::dbscan(input_cloud, eps, min_pts);
        // dbscan(input_cloud, eps, min_pts, number_of_clusters);

        viewer->addPointCloud(input_cloud, "cluster_cloud");

        // 为每个簇计算和显示最小包围盒
        std::vector<std::vector<int>> cluster_indices(number_of_clusters);
        for (size_t i = 0; i < input_cloud->points.size(); ++i) {
            if (input_cloud->points[i].label >= 0) {
                cluster_indices[input_cloud->points[i].label - 1].push_back(i);
            }
        }

        boundingBoxes.clear(); // 清空之前的立方体信息
        // 为每个簇创建包围盒
        for (int i = 0; i < number_of_clusters; ++i) {
            // // 获取当前时间点
            // auto start = std::chrono::high_resolution_clock::now();
            if (cluster_indices[i].size() < 1) continue;

            // 提取当前簇的点云
            pcl::PointCloud<pcl::PointXYZL>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZL>);
            for (const auto& idx : cluster_indices[i]) {
                cluster->points.push_back(input_cloud->points[idx]);
            }

            // 计算包围盒的最小点和最大点
            pcl::PointXYZL min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            

            // 绘制包围盒的12条边
            std::string box_id = "box_" + std::to_string(i);
            viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, 
                1.0, 0.0, 0.0, box_id);
                // 设置立方体的透明度（0.0为完全透明，1.0为完全不透明）
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, box_id);

            // 存储立方体的最小点和最大点
            boundingBoxes[box_id] = {min_pt, max_pt};

            // // 获取结束时间点
            // auto end = std::chrono::high_resolution_clock::now();
            // // 计算时间差并转换为毫秒
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            // std::cout << "执行时间: " << duration.count() << " 毫秒" << std::endl;
        }

        
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(source_cloud, "intensity");

        viewer_source->addPointCloud<pcl::PointXYZI>(source_cloud, intensity_distribution, "source cloud");

        viewer->spinOnce(100);
        viewer_source->spinOnce(100);

        auto currentImageData = std::dynamic_pointer_cast<ImageData>(image_reader->getData());
        cv::Mat display_image = currentImageData->frame.clone();
        // 投影雷达点到图像上
        projectRadarPoints(currentRadarData->points, 
                         display_image,
                         camera_matrix,
                         dist_coeffs,
                         rvec,
                         tvec);
        auto display_imagedata = std::make_shared<ImageData>();
        display_imagedata->frame = display_image;
        displayManager.updateDisplay(display_imagedata); 
        displayManager.renderAll();

    }
}


int main() {
    std::cout << "程序开始运行..." << std::endl;
    std::cout.flush();  // 强制刷新输出缓冲区
    // 设置OpenCV日志级别
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    try {
        // testDisplayManager();
        // testRealTimeRadarDisplay();
        // testPcltoolsCluster();
        
        testDBScan();
        // calib();
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

