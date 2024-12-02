// DataReader.hpp - ���ݶ�ȡ�ӿں�ʵ��
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

// ������������֮ǰ���ǰ������
class SerialPortImpl;

// ���ݶ�ȡ�ӿ�
class IDataReader {
public:
    virtual ~IDataReader() = default;
    virtual bool init() = 0;
    virtual bool readNext() = 0;
    virtual std::shared_ptr<SensorData> getData() = 0;
    virtual bool isEnd() const = 0;
};

// �ļ���ȡ������
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

// ͼ���ļ���ȡ��
class ImageFileReader : public FileReader {
private:
    std::shared_ptr<ImageData> imageData;
    mutable std::shared_mutex dataMutex;

public:
    explicit ImageFileReader(const std::string& path) : FileReader(path) {}
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
};

// �״��ļ���ȡ��
class RadarFileReader : public FileReader {
public:
    enum class Format {
        BIN,
        CSV,
        AUTO    // �Զ�����ļ���ʽ
    };

private:
    std::shared_ptr<RadarData> radarData;   // ��ǰ��ȡ���״�����
    mutable std::shared_mutex dataMutex;
    bool saveEnabled = false;
    std::string savePath;
    std::ofstream outFile;  // ���ڱ����״����ݵ��ļ���
    Format format = Format::AUTO;
    
    // �ڲ���������
    bool readBinData();
    bool readCsvData();
    Format detectFormat(const std::string& filename);

public:
    explicit RadarFileReader(const std::string& path, Format fmt = Format::AUTO) 
        : FileReader(path), format(fmt) {}
    
    bool readNext() override;
    std::shared_ptr<SensorData> getData() override;
    
    // ������ط���
    bool enableSave(const std::string& outputPath, Format saveFormat = Format::BIN);
    void disableSave();
    
    // ��ʽ����
    void setFormat(Format fmt) { format = fmt; }
    Format getFormat() const { return format; }
};

// ���ڶ�ȡ��
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

// ��Ƶ����ȡ��
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
        // ��Ӿ���ʵ��
        return false;  // ��ʱ����ֵ����Ҫ����ʵ���߼��޸�
    }
};


// // tcp�״����ݶ�ȡ��
// class TcpDataReader : public IDataReader {
// private:
//     std::shared_ptr<TcpCommandHandler> handler;  // ���� TcpCommandHandler
    
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

