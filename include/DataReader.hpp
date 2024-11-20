// DataReader.hpp - 数据读取接口和实现
#pragma once
#include "SensorData.hpp"
#include <string>
#include <memory>
#include <fstream>
#include <cstddef>  // for size_t
#ifdef _WIN32
    #include <winsock2.h>
    typedef long long ssize_t;
#else
    #include <sys/types.h>  // for ssize_t on Unix-like systems
#endif

// 在其他类声明之前添加前向声明
class SerialPortImpl;

// 数据读取接口
class IDataReader {
public:
    virtual ~IDataReader() = default;
    virtual bool init() = 0;
    virtual bool readNext() = 0;
    virtual std::shared_ptr<SensorData> getData() = 0;
    virtual bool isEnd() const = 0;
};

// 文件读取器基类
class FileReader : public IDataReader {
protected:
    std::string folderPath;
    std::vector<std::string> fileList;
    size_t currentIndex = 0;

public:
    explicit FileReader(const std::string& path) : folderPath(path) {}
    bool init() override;
    bool isEnd() const override;
};

// 图像文件读取器
class ImageFileReader : public FileReader {
public:
    explicit ImageFileReader(const std::string& path) : FileReader(path) {}
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
};

// 雷达文件读取器
class RadarFileReader : public FileReader {
private:
    bool saveEnabled = false;
    std::string savePath;
    std::ofstream outFile;  // 用于保存雷达数据的文件流

public:
    explicit RadarFileReader(const std::string& path) : FileReader(path) {}
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    
    // 新增保存相关方法
    bool enableSave(const std::string& outputPath);
    void disableSave();
};

// 串口读取器
class SerialReader : public IDataReader {
private:
    std::string portName;
    int baudRate;
    std::unique_ptr<SerialPortImpl> impl;

public:
    SerialReader(const std::string& port, int baud);
    ~SerialReader();
    bool init() override;
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    bool isEnd() const override { return false; }
};

// 视频流读取器
class VideoStreamReader : public IDataReader {
private:
    cv::VideoCapture cap;
    cv::VideoWriter writer;
    std::string streamUrl;
    bool saveEnabled = false;
    std::string savePath;
    int frameWidth = 0;
    int frameHeight = 0;

public:
    explicit VideoStreamReader(const std::string& url) : streamUrl(url) {}
    bool init() override;
    bool grabFrame();
    std::shared_ptr<SensorData> getData() override;
    bool isEnd() const override { return !cap.isOpened(); }
    
    bool enableSave(const std::string& outputPath, double fps = 30.0);
    void disableSave();
    virtual bool readNext() override {
        // 添加具体实现
        return false;  // 临时返回值，需要根据实际逻辑修改
    }
};

class TcpClient {
private:
    std::string serverAddress;
    int port;
    int sockfd;
    bool connected;

public:
    TcpClient(const std::string& address = "192.168.10.117", int port = 50000);
    ~TcpClient();
    
    bool connect();
    bool disconnect();
    bool isConnected() const { return connected; }
    
    // 读写方法
    ssize_t read(uint8_t* buffer, size_t size);
    ssize_t write(const uint8_t* data, size_t size);

    bool setReceiveTimeout(int milliseconds) {
        #ifdef _WIN32
            DWORD timeout = milliseconds;
            return setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, 
                (const char*)&timeout, sizeof(timeout)) == 0;
        #else
            struct timeval tv;
            tv.tv_sec = milliseconds / 1000;
            tv.tv_usec = (milliseconds % 1000) * 1000;
            return setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO,
                (const char*)&tv, sizeof(tv)) == 0;
        #endif
    }
    
    int getSocket() const { return sockfd; }
};

// 然后创建一个专门用于数据读取的包装类
class TcpDataReader : public IDataReader {
private:
    std::shared_ptr<TcpClient> tcpClient;
    
public:
    explicit TcpDataReader(std::shared_ptr<TcpClient> client);
    bool init() override;
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    bool isEnd() const override { return !tcpClient->isConnected(); }
};
