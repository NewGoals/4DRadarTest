// DataReader.hpp - 数据读取接口和实现
#pragma once
#include "SensorData.hpp"
#include <string>

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
    bool init() override {
        // 扫描文件夹获取文件列表
        return true;
    }
    bool isEnd() const override {
        return currentIndex >= fileList.size();
    }
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
public:
    explicit RadarFileReader(const std::string& path) : FileReader(path) {}
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
};

// 串口读取器
class SerialReader : public IDataReader {
private:
    int serialPort;
    std::string portName;
    int baudRate;

public:
    SerialReader(const std::string& port, int baud);
    bool init() override;
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    bool isEnd() const override { return false; }
};

// 视频流读取器
class VideoStreamReader : public IDataReader {
private:
    cv::VideoCapture cap;
    std::string streamUrl;

public:
    explicit VideoStreamReader(const std::string& url) : streamUrl(url) {}
    bool init() override;
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    bool isEnd() const override { return !cap.isOpened(); }
};