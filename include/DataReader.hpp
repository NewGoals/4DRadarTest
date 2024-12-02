// DataReader.hpp - 数据读取接口和实现
#pragma once
#include <string>
#include <memory>
#include <fstream>
#include <cstddef>  // for size_t
#include <filesystem>
#include <shared_mutex>
#include <regex>
#include "SensorData.hpp"
#include "TcpCommandHandler.hpp"

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
private:
    std::shared_ptr<ImageData> imageData;
    mutable std::shared_mutex dataMutex;

public:
    explicit ImageFileReader(const std::string& path) : FileReader(path) {}
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
};

// 雷达文件读取器
class RadarFileReader : public FileReader {
public:
    enum class Format {
        BIN,
        CSV,
        AUTO    // 自动检测文件格式
    };

private:
    std::shared_ptr<RadarData> radarData;   // 当前读取的雷达数据
    mutable std::shared_mutex dataMutex;
    bool saveEnabled = false;
    std::string savePath;
    std::ofstream outFile;  // 用于保存雷达数据的文件流
    Format format = Format::AUTO;
    
    // 内部辅助方法
    bool readBinData();
    bool readCsvData();
    Format detectFormat(const std::string& filename);

public:
    explicit RadarFileReader(const std::string& path, Format fmt = Format::AUTO) 
        : FileReader(path), format(fmt) {}
    
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    
    // 保存相关方法
    bool enableSave(const std::string& outputPath, Format saveFormat = Format::BIN);
    void disableSave();
    
    // 格式设置
    void setFormat(Format fmt) { format = fmt; }
    Format getFormat() const { return format; }
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


// // tcp雷达数据读取器
// class TcpDataReader : public IDataReader {
// private:
//     std::shared_ptr<TcpCommandHandler> handler;  // 改用 TcpCommandHandler
    
// public:
//     explicit TcpDataReader(const std::string& address = "192.168.10.117", int port = 50000)
//         : handler(std::make_shared<TcpCommandHandler>(address, port)) {}
        
//     bool init() override {
//         return handler->connect();
//     }
    
//     bool readNext() override;
//     std::shared_ptr<SensorData> getData() override;
//     bool isEnd() const override { 
//         return !handler || !handler->isConnected(); 
//     }
// };

