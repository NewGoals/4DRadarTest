#include "DataReader.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>

// 平台相关的头文件
#ifdef _WIN32
    #include <windows.h>
#else
    #include <termios.h>
    #include <fcntl.h>
    #include <unistd.h>
#endif

//==============================================================================
// FileReader 基类实现
//==============================================================================
bool FileReader::init() {
    try {
        for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
            if (entry.is_regular_file()) {
                fileList.push_back(entry.path().string());
            }
        }
        return !fileList.empty();
    } catch (const std::exception& e) {
        return false;
    }
}

bool FileReader::isEnd() const {
    return currentIndex >= fileList.size();
}

//==============================================================================
// ImageFileReader 实现
//==============================================================================
bool ImageFileReader::readNext() {
    if (isEnd()) return false;
    currentIndex++;
    return true;
}

std::shared_ptr<SensorData> ImageFileReader::getData() {
    if (currentIndex == 0 || currentIndex > fileList.size()) {
        return nullptr;
    }
    cv::Mat image = cv::imread(fileList[currentIndex - 1]);
    auto data = std::make_shared<SensorData>();
    // 设置图像数据
    return data;
}

//==============================================================================    
// RadarFileReader 实现
//==============================================================================
bool RadarFileReader::readNext() {
    if (isEnd()) return false;
    currentIndex++;
    return true;
}

std::shared_ptr<SensorData> RadarFileReader::getData() {
    if (currentIndex == 0 || currentIndex > fileList.size()) {
        return nullptr;
    }

    // 打开当前雷达数据文件
    std::ifstream inFile(fileList[currentIndex - 1], std::ios::binary);
    if (!inFile) {
        return nullptr;
    }

    auto data = std::make_shared<RadarData>();
    
    // 读取雷达数据
    std::vector<char> buffer;
    inFile.seekg(0, std::ios::end);
    size_t fileSize = inFile.tellg();
    inFile.seekg(0, std::ios::beg);
    
    buffer.resize(fileSize);
    inFile.read(buffer.data(), fileSize);
    
    // 将二进制数据转换为 RadarPoint 结构
    size_t numPoints = fileSize / sizeof(RadarPoint);
    data->points.resize(numPoints);
    memcpy(data->points.data(), buffer.data(), fileSize);
    
    // data->timestamp = /* 从文件名或数据中提取时间戳 */;
    
    // 如果启用了保存功能，将数据写入输出文件
    if (saveEnabled && outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(data->points.data()), fileSize);
    }
    
    return data;
}

bool RadarFileReader::enableSave(const std::string& outputPath) {
    if (saveEnabled) {
        disableSave();  // 如果已经启用，先关闭当前文件
    }
    
    savePath = outputPath;
    outFile.open(savePath, std::ios::binary);
    saveEnabled = outFile.is_open();
    
    return saveEnabled;
}

void RadarFileReader::disableSave() {
    if (outFile.is_open()) {
        outFile.close();
    }
    saveEnabled = false;
}

//==============================================================================
// SerialReader 跨平台实现
//==============================================================================
class SerialPortImpl {
public:
    #ifdef _WIN32
        HANDLE hSerial;
    #else
        int fd;
    #endif

    bool open(const std::string& portName, int baudRate) {
        #ifdef _WIN32
            std::string fullPortName = "\\\\.\\" + portName;
            hSerial = CreateFileA(fullPortName.c_str(),
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                0,
                                OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL,
                                0);

            if (hSerial == INVALID_HANDLE_VALUE) return false;

            DCB dcbSerialParams = {0};
            dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
            
            if (!GetCommState(hSerial, &dcbSerialParams)) {
                CloseHandle(hSerial);
                return false;
            }

            dcbSerialParams.BaudRate = baudRate;
            dcbSerialParams.ByteSize = 8;
            dcbSerialParams.StopBits = ONESTOPBIT;
            dcbSerialParams.Parity = NOPARITY;

            if (!SetCommState(hSerial, &dcbSerialParams)) {
                CloseHandle(hSerial);
                return false;
            }
            return true;
        #else
            fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY);
            if (fd < 0) return false;

            struct termios tty;
            memset(&tty, 0, sizeof(tty));
            if (tcgetattr(fd, &tty) != 0) return false;

            cfsetospeed(&tty, baudRate);
            cfsetispeed(&tty, baudRate);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;

            if (tcsetattr(fd, TCSANOW, &tty) != 0) return false;
            return true;
        #endif
    }

    void close() {
        #ifdef _WIN32
            if (hSerial != INVALID_HANDLE_VALUE) {
                CloseHandle(hSerial);
                hSerial = INVALID_HANDLE_VALUE;
            }
        #else
            if (fd >= 0) {
                ::close(fd);
                fd = -1;
            }
        #endif
    }
};

SerialReader::SerialReader(const std::string& port, int baud) 
    : portName(port), baudRate(baud) {
    impl = std::make_unique<SerialPortImpl>();
}

SerialReader::~SerialReader() {
    if (impl) {
        impl->close();
    }
}

bool SerialReader::init() {
    return impl->open(portName, baudRate);
}

bool SerialReader::readNext() {
    // 实现读取逻辑
    return true;
}

std::shared_ptr<SensorData> SerialReader::getData() {
    auto data = std::make_shared<SensorData>();
    // 处理数据
    return data;
}

//==============================================================================
// VideoStreamReader 实现
//==============================================================================
bool VideoStreamReader::init() {
    if (!cap.open(streamUrl)) {
        return false;
    }

    // 尝试读取第一帧以获取尺寸
    cv::Mat frame;
    if (cap.read(frame) && frameWidth == 0 && frameHeight == 0) {
        frameWidth = frame.cols;
        frameHeight = frame.rows;
    } else {
        return false; // 无法读取视频流
    }

    return true;
}

bool VideoStreamReader::grabFrame() {
    if (!cap.isOpened()) return false;
    
    // 只获取帧，不解码，这样更快
    if (!cap.grab()) return false;
    
    // 如果需要保存，则需要解码并保存
    if (saveEnabled && writer.isOpened()) {
        cv::Mat frame;
        if (cap.retrieve(frame)) {
            writer.write(frame);
        }
    }
    
    return true;
}

std::shared_ptr<SensorData> VideoStreamReader::getData() {
    cv::Mat frame;
    cap.retrieve(frame);
    
    auto data = std::make_shared<ImageData>();  // 假设 ImageData 是 SensorData 的子类
    data->frame = frame;
    data->timestamp = cap.get(cv::CAP_PROP_POS_MSEC);  // 保持毫秒单位
    
    return data;
}

bool VideoStreamReader::enableSave(const std::string& outputPath, double fps) {
    savePath = outputPath;
    saveEnabled = true;

    // 检查 frameWidth 和 frameHeight 是否已被设置
    if (frameWidth == 0 && frameHeight == 0) {
        return false; // 或者抛出异常
    }

    // 打开视频写入器
    writer.open(savePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight));
    bool flag = writer.isOpened();
    return writer.isOpened(); // 返回 writer 是否成功打开
}

void VideoStreamReader::disableSave() {
    if (writer.isOpened()) {
        writer.release();
    }
    saveEnabled = false;
} 