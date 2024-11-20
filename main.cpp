#include <iostream>
#include <string>
#include <memory>
#include "DataReaderFactory.hpp"
#include "TcpCommandHandler.hpp"
#include <windows.h>

// 用于打印十六进制数据
void printHex(const std::vector<uint8_t>& data) {
    for (uint8_t byte : data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

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

void testTcpClient() {
    // 创建TCP客户端实例
    TcpClient client("192.168.10.117", 50000);
    
    // 尝试连接服务器
    if (!client.connect()) {
        std::cout << "连接服务器失败！" << std::endl;
        return;
    }
    
    std::cout << "成功连接到服务器！" << std::endl;
    
    // 准备发送的测试数据
    std::vector<uint8_t> sendData = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    // 发送数据
    ssize_t sendResult = client.write(sendData.data(), sendData.size());
    if (sendResult < 0) {
        std::cout << "发送数据失败！" << std::endl;
        client.disconnect();
        return;
    }
    
    std::cout << "成功发送 " << sendResult << " 字节的数据" << std::endl;
    
    // 准备接收缓冲区
    std::vector<uint8_t> recvBuffer(1024);
    
    // 接收数
    ssize_t recvResult = client.read(recvBuffer.data(), recvBuffer.size());
    if (recvResult < 0) {
        std::cout << "接收数据失败！" << std::endl;
    } else if (recvResult > 0) {
        std::cout << "接收到 " << recvResult << " 字节的数据：";
        // 打印接收到的数据（十六进制格式）
        for (size_t i = 0; i < recvResult; ++i) {
            printf("%02X ", recvBuffer[i]);
        }
        std::cout << std::endl;
    }
    
    // 等待一段时间
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 断开连接
    client.disconnect();
    std::cout << "已断开连接" << std::endl;
}

void testTcpCommandHandler() {
    // 创建TCP客户端和命令处理器
    TcpClient client("192.168.10.117", 50000);
    TcpCommandHandler handler(client);
    
    // 连接服务器
    if (!client.connect()) {
        std::cout << "连接服务器失败！" << std::endl;
        return;
    }
    std::cout << "成功连接到服务器！" << std::endl;
    
    // 设置接收超时
    client.setReceiveTimeout(100);  // 100ms超时
    
    // 测试发送命令
    uint8_t srcAddr = 0x10;
    uint8_t destAddr = 0x90;
    CommandCode cmd = CommandCode::READ_RADAR_STATUS;
    std::vector<uint8_t> testData = {};
    
    std::cout << "\n发送测试命令..." << std::endl;
    if (handler.sendFrame(srcAddr, destAddr, cmd, testData)) {
        std::cout << "命令发送成功！" << std::endl;
    } else {
        std::cout << "命令发送失败！" << std::endl;
        client.disconnect();
        return;
    }
    
    // 接收和处理数据
    auto lastTime = std::chrono::steady_clock::now();
    int frameCount = 0;
    
    std::cout << "\n开始接收数据..." << std::endl;
    while (true) {
        try {
            ProtocolFrame receivedFrame;
            if (handler.receiveFrame(receivedFrame)) {
                frameCount++;
                
                // 计算时间间隔
                auto currentTime = std::chrono::steady_clock::now();
                auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                    currentTime - lastTime).count();
                lastTime = currentTime;
                
                // 输出帧信息
                std::cout << "\n接收到第 " << frameCount << " 帧数据:" << std::endl;
                std::cout << "命令码: 0x" << std::hex 
                          << static_cast<int>(static_cast<uint8_t>(receivedFrame.header.command))
                          << std::dec << std::endl;
                std::cout << "源地址: 0x" << std::hex 
                          << static_cast<int>(receivedFrame.header.srcAddr) << std::endl;
                std::cout << "目标地址: 0x" 
                          << static_cast<int>(receivedFrame.header.destAddr) << std::dec << std::endl;
                std::cout << "数据长度: " 
                          << (receivedFrame.header.lengthLow | (receivedFrame.header.lengthHigh << 8)) 
                          << " 字节" << std::endl;
                std::cout << "时间间隔: " << interval << "ms" << std::endl;
                
                // 打印数据内容（十六进制）
                if (!receivedFrame.data.empty()) {
                    std::cout << "数据内容: ";
                    for (uint8_t byte : receivedFrame.data) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                                  << static_cast<int>(byte) << " ";
                    }
                    std::cout << std::dec << std::endl;
                }
            }
            
            // 检查退出条件
            if (GetAsyncKeyState(VK_ESCAPE)) {
                std::cout << "\n检测到ESC键, 退出测试" << std::endl;
                break;
            }
            
            // 避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << std::endl;
            break;
        }
    }
    
    // 输出统计信息
    std::cout << "\n测试结束" << std::endl;
    std::cout << "总共接收: " << frameCount << " 帧数据" << std::endl;
    
    // 断开连接
    client.disconnect();
    std::cout << "已断开连接" << std::endl;
}

void testRadar() {
    // 创建TCP客户端和命令处理器
    TcpClient client("192.168.10.117", 50000);
    TcpCommandHandler handler(client);
    
    // 连接服务器
    if (!client.connect()) {
        std::cout << "连接服务器失败！" << std::endl;
        return;
    }
    std::cout << "成功连接到服务器！" << std::endl;
    
    // 设置接收超时
    client.setReceiveTimeout(100);  // 100ms超时

    // 接收和处理数据
    auto lastTime = std::chrono::steady_clock::now();
    int frameCount = 0;
    
    std::cout << "\n开始接收数据..." << std::endl;
    while (true) {
        try {
            ProtocolFrame receivedFrame;
            if (handler.receiveFrame(receivedFrame)) {
                frameCount++;
                
                // 计算时间间隔
                auto currentTime = std::chrono::steady_clock::now();
                auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                    currentTime - lastTime).count();
                lastTime = currentTime;

                CommandParseResult result{std::monostate{}};  // 使用花括号初始化
                if (Protocol::parseCommandData(receivedFrame, result)) {
                    if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
                        // 使用 targets 数据 
                        std::cout << "接收到 " << targets->size() << " 个目标" << std::endl;

                        // 将该数据保存到文件，使用时间戳
                        
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "错误: " << e.what() << std::endl;
            break;
        }
    }
    // 输出统计信息
    std::cout << "\n测试结束" << std::endl;
    std::cout << "总共接收: " << frameCount << " 帧数据" << std::endl;
    
    // 断开连接
    client.disconnect();
    std::cout << "已断开连接" << std::endl;
}

int main() {
    try {
        testRadar();
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
