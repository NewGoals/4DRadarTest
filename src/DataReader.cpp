#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN  // ������������Windowsͷ�ļ�
    #include <WinSock2.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

#include <iostream>
#include "DataReader.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>

//==============================================================================
// FileReader ����ʵ��
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
// ImageFileReader ʵ��
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
    // ����ͼ������
    return data;
}

//==============================================================================    
// RadarFileReader ʵ��
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

    // �򿪵�ǰ�״������ļ�
    std::ifstream inFile(fileList[currentIndex - 1], std::ios::binary);
    if (!inFile) {
        return nullptr;
    }

    auto data = std::make_shared<RadarData>();
    
    // ��ȡ�״�����
    std::vector<char> buffer;
    inFile.seekg(0, std::ios::end);
    size_t fileSize = inFile.tellg();
    inFile.seekg(0, std::ios::beg);
    
    buffer.resize(fileSize);
    inFile.read(buffer.data(), fileSize);
    
    // ������������ת��Ϊ RadarPoint �ṹ
    size_t numPoints = fileSize / sizeof(RadarPoint);
    data->points.resize(numPoints);
    memcpy(data->points.data(), buffer.data(), fileSize);
    
    // data->timestamp = /* ���ļ�������������ȡʱ��� */;
    
    // ��������˱��湦�ܣ�������д������ļ�
    if (saveEnabled && outFile.is_open()) {
        outFile.write(reinterpret_cast<const char*>(data->points.data()), fileSize);
    }
    
    return data;
}

bool RadarFileReader::enableSave(const std::string& outputPath) {
    if (saveEnabled) {
        disableSave();  // ����Ѿ����ã��ȹرյ�ǰ�ļ�
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
// SerialReader ��ƽ̨ʵ��
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
    // ʵ�ֶ�ȡ�߼�
    return true;
}

std::shared_ptr<SensorData> SerialReader::getData() {
    auto data = std::make_shared<SensorData>();
    // ��������
    return data;
}

//==============================================================================
// VideoStreamReader ʵ��
//==============================================================================
bool VideoStreamReader::init() {
    if (!cap.open(streamUrl)) {
        return false;
    }

    // ���Զ�ȡ��һ֡�Ի�ȡ�ߴ�
    cv::Mat frame;
    if (cap.read(frame)) {
        if(frameWidth == 0 && frameHeight == 0) {
            frameWidth = frame.cols;
            frameHeight = frame.rows;
        }
    }
    else {
        return false;
    }

    return true;
}

bool VideoStreamReader::grabFrame() {
    if (!cap.isOpened()) return false;
    
    // ֻ��ȡ֡�������룬��������
    if (!cap.grab()) return false;
    
    // �����Ҫ���棬����Ҫ���벢����
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
    
    auto data = std::make_shared<ImageData>();  // ���� ImageData �� SensorData ������
    data->frame = frame;
    data->timestamp = static_cast<int64_t>(cap.get(cv::CAP_PROP_POS_MSEC));  // ��ʽת��Ϊ int64_t
    
    return data;
}

bool VideoStreamReader::enableSave(const std::string& outputPath, double fps) {
    savePath = outputPath;
    saveEnabled = true;

    // ��� frameWidth �� frameHeight ���ѱ�����
    if (frameWidth == 0 && frameHeight == 0) {
        return false; // �����׳��쳣
    }

    // ����Ƶд����
    writer.open(savePath, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight));
    bool flag = writer.isOpened();
    return writer.isOpened(); // ���� writer �Ƿ�ɹ���
}

void VideoStreamReader::disableSave() {
    if (writer.isOpened()) {
        writer.release();
    }
    saveEnabled = false;
} 

