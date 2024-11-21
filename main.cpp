#include <iostream>
#include <string>
#include <memory>
#include "DataReaderFactory.hpp"
#include "TcpCommandHandler.hpp"
#include "SynchronizedCollector.hpp"
#include <windows.h>
#include <filesystem>


// ���ڴ�ӡʮ����������
void printHex(const std::vector<uint8_t>& data) {
    for (uint8_t byte : data) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

void testVideoReader() {
    // ������Ƶ�ļ�·��
    std::string videoPath = "E:/dataset/How Tower Cranes Build Themselves1080p.mp4";  // �滻Ϊ�����Ƶ·��
    std::string savePath = "test_output.avi";  // ����·��
    
    // ʹ�ù����ഴ����Ƶ��ȡ��
    auto reader = DataReaderFactory::createReader(ReaderType::VIDEO_STREAM, videoPath);
    if (!reader) {
        std::cerr << "Failed to create video reader!" << std::endl;
        return;
    }
    
    // ת��ΪVideoStreamReader��ʹ���ض�����
    auto videoReader = std::dynamic_pointer_cast<VideoStreamReader>(reader);
    if (!videoReader) {
        std::cerr << "Failed to cast to VideoStreamReader!" << std::endl;
        return;
    }
    
    // ��ʼ��
    if (!videoReader->init()) {
        std::cerr << "Failed to initialize video reader!" << std::endl;
        return;
    }
    
    // ���ñ���
    if (!videoReader->enableSave(savePath, 30.0)) {
        std::cerr << "Failed to enable video saving!" << std::endl;
        return;
    }
    
    int frameCount = 0;
    // ��ȡ��������Ƶ֡
    while (videoReader->grabFrame()) {
        auto data = videoReader->getData();
        if (data) {
            frameCount++;
            std::cout << "Frame " << frameCount 
                      << ", Timestamp: " << data->timestamp << "ms" << std::endl;
            
            // ��ʾ��ǰ֡
            cv::Mat& frame = std::dynamic_pointer_cast<ImageData>(data)->frame;
            cv::imshow("Video", frame);
            if (cv::waitKey(1) == 27) {  // ESC���˳�
                break;
            }
        }
    }
    
    videoReader->disableSave();
    cv::destroyAllWindows();
    std::cout << "Processed " << frameCount << " frames" << std::endl;
}

void testTcpClient() {
    // ����TCP�ͻ���ʵ��
    TcpClient client("192.168.10.117", 50000);
    
    // �������ӷ�����
    if (!client.connect()) {
        std::cout << "���ӷ�����ʧ�ܣ�" << std::endl;
        return;
    }
    
    std::cout << "�ɹ����ӵ���������" << std::endl;
    
    // ׼�����͵Ĳ�������
    std::vector<uint8_t> sendData = {0x01, 0x02, 0x03, 0x04, 0x05};
    
    // ��������
    ssize_t sendResult = client.write(sendData.data(), sendData.size());
    if (sendResult < 0) {
        std::cout << "��������ʧ�ܣ�" << std::endl;
        client.disconnect();
        return;
    }
    
    std::cout << "�ɹ����� " << sendResult << " �ֽڵ�����" << std::endl;
    
    // ׼�����ջ�����
    std::vector<uint8_t> recvBuffer(1024);
    
    // ������
    ssize_t recvResult = client.read(recvBuffer.data(), recvBuffer.size());
    if (recvResult < 0) {
        std::cout << "��������ʧ�ܣ�" << std::endl;
    } else if (recvResult > 0) {
        std::cout << "���յ� " << recvResult << " �ֽڵ����ݣ�";
        // ��ӡ���յ������ݣ�ʮ�����Ƹ�ʽ��
        for (ssize_t i = 0; i < recvResult; ++i) {
            printf("%02X ", recvBuffer[i]);
        }
        std::cout << std::endl;
    }
    
    // �ȴ�һ��ʱ��
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // �Ͽ�����
    client.disconnect();
    std::cout << "�ѶϿ�����" << std::endl;
}

// �����״� 
void testRadar() {
    try {
        // �����������
        TcpCommandHandler handler("192.168.10.117", 50000);
        
        // ��������Ŀ¼
        std::string saveDir = "radar_data";
        if (!std::filesystem::exists(saveDir)) {
            std::filesystem::create_directory(saveDir);
        }
        
        // �����״�
        if (!handler.connect()) {
            std::cerr << "�޷����ӵ��״��豸" << std::endl;
            return;
        }
        std::cout << "�ɹ����ӵ��״��豸" << std::endl;

        // ��ѭ��
        int count = 0;
        const int totalFrames = 200;
        const int printInterval = 10;  // ÿ10֡��ӡһ��
        auto lastTime = std::chrono::steady_clock::now();
        auto lastPrintTime = std::chrono::steady_clock::now();
        
        while (count < totalFrames) {
            CommandParseResult result;
            if (handler.receiveAndParseFrame(result)) {
                if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
                    // ����ʱ����ļ���
                    auto now = std::chrono::system_clock::now();
                    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()).count();
                    std::string filename = saveDir + "/radar_" + std::to_string(timestamp) + ".csv";
                    
                    // ���浱ǰ֡����
                    if (handler.startRecording(filename)) {
                        // std::cout << "����֡���ݵ�: " << filename << std::endl;
                        // std::cout << "���յ�Ŀ������: " << targets->size() << std::endl;
                        handler.saveTargetData(*targets);
                        handler.stopRecording();
                    }
                    
                    // ÿ��һ��֡���Ŵ�ӡ����
                    if (count % printInterval == 0) {
                        auto now = std::chrono::steady_clock::now();
                        auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                            now - lastPrintTime).count();
                        lastPrintTime = now;
                        
                        float progress = (count + 1) * 100.0f / totalFrames;
                        std::cout << "\r�ɼ�����: " << std::fixed << std::setprecision(1) 
                                  << progress << "% (" << (count + 1) << "/" << totalFrames 
                                  << "), ƽ���ɼ����: " << interval / printInterval << "ms" 
                                  << std::flush;  // ʹ��\r��std::flushʵ��ԭ�ظ���
                    }
                }
                count++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << std::endl;  // ��ɺ���
        
        handler.disconnect();
        std::cout << "������ɣ��ѶϿ�����" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "�״�����쳣: " << e.what() << std::endl;
    }
}

void testMultiSourceCapture() {
    try {
        SynchronizedCollector collector;
        std::cout << "�����ɼ����ɹ�" << std::endl;
        
        // ���������Դ���״
        auto radarSource = std::make_unique<RadarSource>("192.168.10.117", 50000, "radar");
        if (!radarSource->init()) {
            throw std::runtime_error("�״��ʼ��ʧ��");
        }
        collector.addSource(std::move(radarSource), true);
        std::cout << "����״�Դ�ɹ�" << std::endl;
        
        // ��Ӵ�����Դ����Ƶ��
        auto videoSource = std::make_unique<VideoSource>(
            "E:/dataset/How Tower Cranes Build Themselves1080p.mp4", "camera1");
        if (!videoSource->init()) {
            throw std::runtime_error("��ƵԴ��ʼ��ʧ��");
        }
        collector.addSource(std::move(videoSource), false);
        std::cout << "�����ƵԴ�ɹ�" << std::endl;
        
        collector.start();
        
        while (true) {
            if (cv::waitKey(1000) == 27) {
                std::cout << "��⵽ESC����׼���˳�..." << std::endl;
                break;
            }
            collector.printStats();
        }
        
        collector.stop();
        std::cout << "�ɼ���ֹͣ" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "��Դ�ɼ��쳣: " << e.what() << std::endl;
    }
}


int main() {
    // ����OpenCV��־����
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    try {
        testMultiSourceCapture();
    } catch (const std::exception& e) {
        std::cerr << "�����쳣: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
