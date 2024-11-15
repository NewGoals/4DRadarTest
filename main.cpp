#include <iostream>
#include <string>
#include <memory>
#include "DataReaderFactory.hpp"

void testVideoReader() {
    // 本地视频文件路径
    std::string videoPath = "E:/dataset/How Tower Cranes Build Themselves1080p.mp4";  // 替换为你的视频路径
    std::string savePath = "test_output.avi";  // 保存路径
    
    // 使用工厂类创建视频读取器
    auto reader = DataReaderFactory::createReader(ReaderType::VIDEO_STREAM, videoPath);
    if (!reader) {
        std::cerr << "Failed to create video reader!" << std::endl;
        return;
    }
    
    // 转换为VideoStreamReader以使用特定功能
    auto videoReader = std::dynamic_pointer_cast<VideoStreamReader>(reader);
    if (!videoReader) {
        std::cerr << "Failed to cast to VideoStreamReader!" << std::endl;
        return;
    }
    
    // 初始化
    if (!videoReader->init()) {
        std::cerr << "Failed to initialize video reader!" << std::endl;
        return;
    }
    
    // 启用保存
    if (!videoReader->enableSave(savePath, 30.0)) {
        std::cerr << "Failed to enable video saving!" << std::endl;
        return;
    }
    
    int frameCount = 0;
    // 读取并保存视频帧
    while (videoReader->grabFrame()) {
        auto data = videoReader->getData();
        if (data) {
            frameCount++;
            std::cout << "Frame " << frameCount 
                      << ", Timestamp: " << data->timestamp << "ms" << std::endl;
            
            // 显示当前帧
            cv::Mat& frame = std::dynamic_pointer_cast<ImageData>(data)->frame;
            cv::imshow("Video", frame);
            if (cv::waitKey(1) == 27) {  // ESC键退出
                break;
            }
        }
    }
    
    videoReader->disableSave();
    cv::destroyAllWindows();
    std::cout << "Processed " << frameCount << " frames" << std::endl;
}

int main() {
    // std::cout << "FFmpeg support: " << cv::getBuildInformation() << std::endl;
    try {
        testVideoReader();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
