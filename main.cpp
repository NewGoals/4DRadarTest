#include "TcpCommandHandler.hpp"  // 包含了所有Windows相关定义
#include "SensorData.hpp"
#include "DataReaderFactory.hpp"
#include "DisplayManager.hpp"   // 其中既有eigen又有opencv
#include "SynchronizedCollector.hpp"
#include <filesystem>
#include <opencv2/core/utils/logger.hpp>


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

// // 测试雷达 
// void testRadar() {
//     try {
//         // 创建命令处理器
//         TcpCommandHandler handler("192.168.10.117", 50000);
        
//         // 创建保存目录
//         std::string saveDir = "radar_data";
//         if (!std::filesystem::exists(saveDir)) {
//             std::filesystem::create_directory(saveDir);
//         }
        
//         // 连接雷达
//         if (!handler.connect()) {
//             std::cerr << "无法连接到雷达设备" << std::endl;
//             return;
//         }
//         std::cout << "成功连接到雷达设备" << std::endl;

//         // 主循环
//         int count = 0;
//         const int totalFrames = 200;
//         const int printInterval = 10;  // 每10帧打印一次
//         auto lastTime = std::chrono::steady_clock::now();
//         auto lastPrintTime = std::chrono::steady_clock::now();
        
//         while (count < totalFrames) {
//             CommandParseResult result;
//             if (handler.receiveAndParseFrame(result)) {
//                 if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
//                     // 生成时间戳文件名
//                     auto now = std::chrono::system_clock::now();
//                     auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
//                         now.time_since_epoch()).count();
//                     std::string filename = saveDir + "/radar_" + std::to_string(timestamp) + ".csv";
                    
//                     // 保存当前帧数据
//                     if (handler.startRecording(filename)) {
//                         // std::cout << "保存帧数据到: " << filename << std::endl;
//                         // std::cout << "接收到目标数量: " << targets->size() << std::endl;
//                         handler.saveTargetData(*targets);
//                         handler.stopRecording();
//                     }
                    
//                     // 每隔一定帧数才打印进度
//                     if (count % printInterval == 0) {
//                         auto now = std::chrono::steady_clock::now();
//                         auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
//                             now - lastPrintTime).count();
//                         lastPrintTime = now;
                        
//                         float progress = (count + 1) * 100.0f / totalFrames;
//                         std::cout << "\r采集进度: " << std::fixed << std::setprecision(1) 
//                                   << progress << "% (" << (count + 1) << "/" << totalFrames 
//                                   << "), 平均采集间隔: " << interval / printInterval << "ms" 
//                                   << std::flush;  // 使用\r和std::flush实现原地更新
//                     }
//                 }
//                 count++;
//             }
//             std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         }
//         std::cout << std::endl;  // 完成后换行
        
//         handler.disconnect();
//         std::cout << "测试完成，已断开连接" << std::endl;
        
//     } catch (const std::exception& e) {
//         std::cerr << "雷达测试异常: " << e.what() << std::endl;
//     }
// }

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

// void testRadarFileReader() {
//     std::cout << "开始测试雷达数据读取..." << std::endl;
    
//     // 创建雷达数据读取器
//     auto reader = DataReaderFactory::createReader(
//         ReaderType::RADAR_FILE,
//         "E:/dataset/radar_data_20241122/sync_data_20241122_104031/radar"  // 雷达数据文件夹路径
//     );
    
//     if (!reader) {
//         std::cerr << "创建读取器失败" << std::endl;
//         return;
//     }
    
//     // 初始化读取器
//     if (!reader->init()) {
//         std::cerr << "初始化读取器失败" << std::endl;
//         return;
//     }
    
//     // 读取并打印数据
//     int frameCount = 0;
//     while (!reader->isEnd() && frameCount < 100) {  // 读取10帧数据
//         if (reader->readNext()) {
//             auto data = std::dynamic_pointer_cast<RadarData>(reader->getData());
//             if (data) {
//                 std::cout << "帧 " << frameCount 
//                          << " 时间戳: " << data->timestamp
//                          << " 点数: " << data->points.size() << std::endl;
                
//                 // 打印第一个点的数据（如果有）
//                 if (!data->points.empty()) {
//                     const auto& p = data->points[0];
//                     std::cout << "第一个点: x=" << p.x 
//                              << ", y=" << p.y 
//                              << ", z=" << p.z 
//                              << ", rcs=" << p.rcs 
//                              << ", v_r=" << p.v_r << std::endl;
//                 }
//             }
//             frameCount++;
//         }
//     }
    
//     std::cout << "测试完成，共读取 " << frameCount << " 帧数据" << std::endl;
// }

void testDisplayManager(){
    try
    {
        std::cout << "开始测试显示管理器..." << std::endl;
        DisplayManager displayManager;

        // 添加可视化器
        displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
        
        // 设置窗口布局
        displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);

        // 从文件中读取数据
        auto reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE, "E:/dataset/radar_data_20241122/sync_data_20241122_104031/radar");
        if (!reader->init()) {
            std::cerr << "初始化读取器失败" << std::endl;
            return;
        }

        // 读取数据
        while (!reader->isEnd()) {
            if (reader->readNext()) {
                displayManager.updateDisplay(reader->getData());
                displayManager.renderAll();
                if (cv::waitKey(1) == 'q') break;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << "testDisplayManager 异常: " << e.what() << '\n';
    }
    
    
}



void testRealTimeRadarDisplay(){
    try{
        SynchronizedCollector collector;
        std::cout << "创建采集器成功" << std::endl;
        
        // 添加主数据源（雷达）
        auto radarSource = std::make_unique<RadarSource>("192.168.10.117", 50000, "radar");
        if (!radarSource->init()) {
            throw std::runtime_error("雷达初始化失败");
        }
        collector.addSource(std::move(radarSource), true);
        std::cout << "添加雷达源成功" << std::endl;

        // 添加从视频源
        // auto videoSource = std::make_unique<VideoSource>(
        //     "E:/dataset/How Tower Cranes Build Themselves1080p.mp4", "camera");
        auto videoSource = std::make_unique<VideoSource>(
            "rtsp://127.0.0.1:8554/camera_test", "camera");
        if (!videoSource->init()) {
            throw std::runtime_error("视频源初始化失败");
        }
        collector.addSource(std::move(videoSource), false);
        std::cout << "添加视频源成功" << std::endl;

        // 设置保存，雷达保存，视频不保存
        collector.setSaveConfig(true, true);

        // 创建可视化器
        DisplayManager displayManager;
        displayManager.addVisualizer(DisplayManager::DisplayType::POINT_CLOUD, std::make_shared<PointCloudVisualizer>());
        displayManager.addVisualizer(DisplayManager::DisplayType::IMAGE, std::make_shared<ImageVisualizer>());

        // 设置窗口布局
        displayManager.setLayout(DisplayManager::DisplayType::POINT_CLOUD, 100, 100, 800, 600);
        displayManager.setLayout(DisplayManager::DisplayType::IMAGE, 100, 700, 800, 600);
        // 启动线程
        collector.start();

        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(5));  // 改为5ms
            collector.printStats();
            displayManager.updateDisplay(collector.getMainSourceData());
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


struct InitDebug {
    InitDebug() {
        AllocConsole();
        FILE* dummy;
        freopen_s(&dummy, "CONOUT$", "w", stdout);
        freopen_s(&dummy, "CONOUT$", "w", stderr);
        std::cout << "全局初始化开始" << std::endl;
    }
} g_init_debug;

int main() {
    std::cout << "程序开始运行..." << std::endl;
    std::cout.flush();  // 强制刷新输出缓冲区
    // 设置OpenCV日志级别
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    try {
        testRealTimeRadarDisplay();
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}

