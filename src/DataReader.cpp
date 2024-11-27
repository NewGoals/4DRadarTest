// #ifdef _WIN32
//     #define WIN32_LEAN_AND_MEAN  // 避免包含多余的Windows头文件
//     #include <WinSock2.h>
//     #include <WS2tcpip.h>
//     #pragma comment(lib, "ws2_32.lib")
// #else
//     #include <sys/socket.h>
//     #include <netinet/in.h>
//     #include <arpa/inet.h>
//     #include <unistd.h>
// #endif

#include <iostream>
#include "DataReader.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>

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
        std::cerr << "Error: " << e.what() << std::endl;
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
/// @brief 读取下一帧雷达数据
bool RadarFileReader::readNext() {
    if (isEnd()) return false;
    
    // 根据文件格式选择相应的读取方法
    Format currentFormat = (format == Format::AUTO) ? 
        detectFormat(fileList[currentIndex]) : format;

    // 打印雷达文件
    std::cout << "读取雷达文件: " << fileList[currentIndex] << std::endl;
        
    bool success = false;
    switch (currentFormat) {
        case Format::BIN:
            success = readBinData();
            break;
        case Format::CSV:
            success = readCsvData();
            break;
        default:
            std::cerr << "不支持的文件格式" << std::endl;
            return false;
    }
    
    // 不管正确与否都要下一帧，防止无限循环卡死
    currentIndex++;

    return success;
}

/// @brief 获取当前帧雷达数据
/// @return 当前帧雷达数据(SensorData 指针)
std::shared_ptr<SensorData> RadarFileReader::getData() {
    if (!radarData) return nullptr;
    return radarData;
}

bool RadarFileReader::readBinData() {
    std::ifstream inFile(fileList[currentIndex], std::ios::binary);
    if (!inFile) return false;

    // 读取二进制数据的实现...
    return true;
}

/// @brief 读取原始CSV格式雷达数据
bool RadarFileReader::readCsvData() {
    std::ifstream inFile(fileList[currentIndex]);
    if (!inFile) return false;

    radarData = std::make_shared<RadarData>();
    std::string line;

    // 读取验证标题行
    std::getline(inFile, line);
    if (line != "timestamp,type,x,y,z,range,speed,peak,doppler,snr"){
        std::cerr << "CSV格式不匹配" << std::endl;
        return false;
    }
    
    // 读取数据行
    while (std::getline(inFile, line)) {
        std::stringstream ss(line);
        std::string value;
        RadarPoint point;

        // timestamp
        std::getline(ss, value, ',');
        radarData->timestamp = std::stoll(value);

        // type
        std::getline(ss, value, ',');

        // x, y, z
        std::getline(ss, value, ',');
        point.x = std::stof(value);
        std::getline(ss, value, ',');
        point.y = std::stof(value);
        std::getline(ss, value, ',');
        point.z = std::stof(value);

        // range
        std::getline(ss, value, ',');

        // speed
        std::getline(ss, value, ',');
        point.v_r = std::stof(value);

        // peak
        std::getline(ss, value, ',');
        point.rcs = std::stof(value);

        // doppler, snr
        std::getline(ss, value, ',');
        std::getline(ss, value, ',');

        radarData->points.push_back(point);
    }
    
    return true;
}

RadarFileReader::Format RadarFileReader::detectFormat(const std::string& filename) {
    // 根据文件扩展名判断格式
    std::string ext = std::filesystem::path(filename).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if (ext == ".bin") return Format::BIN;
    if (ext == ".csv") return Format::CSV;
    
    // 默认返回二进制格式
    return Format::BIN;
}

bool RadarFileReader::enableSave(const std::string& outputPath, Format saveFormat) {
    if (saveEnabled) {
        disableSave();
    }
    
    savePath = outputPath;
    std::ios_base::openmode mode = (saveFormat == Format::BIN) ? 
        std::ios::binary : std::ios::out;
    
    outFile.open(savePath, mode);
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
        std::cerr << "无法打开video stream: " << streamUrl << std::endl;
        return false;
    }

    // 尝试读取第一帧以获取尺寸
    cv::Mat frame;
    if (cap.read(frame)) {
        if(frameWidth == 0 && frameHeight == 0) {
            frameWidth = frame.cols;
            frameHeight = frame.rows;
        }
    }
    else {
        std::cerr << "无法读取video stream: " << streamUrl << std::endl;
        return false;
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
    data->timestamp = static_cast<int64_t>(cap.get(cv::CAP_PROP_POS_MSEC));  // 显式转换为 int64_t
    
    return data;
}

bool VideoStreamReader::enableSave(const std::string& outputPath, double fps) {
    savePath = outputPath;
    saveEnabled = true;

    // 检查 frameWidth 和 frameHeight 否已被设置
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

