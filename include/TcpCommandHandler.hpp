#pragma once
#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <WinSock2.h>  // 必须在 windows.h 之前
    #include <Windows.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    typedef long long ssize_t;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

#include <iostream>
#include <cstdint>
#include <vector>
#include <memory>
#include <functional>
#include <variant>
#include <atomic>
#include <fstream>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>


// 命令码枚举定义
enum class CommandCode : uint8_t {
    FACTORY_RESET       = 0x01,    // 恢复出厂设置
    ADD_COORDINATE      = 0x03,    // 添加坐标设置
    NETWORK_CONFIG      = 0x04,    // 更改网络设置
    BROADCAST_NETWORK   = 0x05,    // 广播网络参数
    HEARTBEAT_CONFIG    = 0x09,    // 心跳时间设置
    READ_RADAR_STATUS   = 0x0A,    // 读取雷达状态
    SYSTEM_TIME         = 0x24,    // 系统时间设置
    SAVE_PARAMS         = 0x88,    // 保存参数
    HEARTBEAT           = 0xA4,    // 心跳包
    TARGET_INFO         = 0xA8,    // 目标信息传输
    FIRMWARE_UPDATE     = 0xCC,    // 固件更新
    FILE_TRANSFER       = 0xD7,    // 文件传输过程
    FILE_TRANSFER_END   = 0xD8,    // 文件传输结束
    RADAR_ANGLE_CORRECT = 0xD9,    // 雷达角度校正
    GET_ANGLE_STATUS    = 0xDA,    // 获取雷达角度校正状态
    SYSTEM_RESTART      = 0xDB,    // 重启系统
    READ_PARAMS         = 0xE0,    // 参数读取
    SET_PARAMS          = 0xE1,    // 参数设置
    TEMP_SET_PARAMS     = 0xE3,    // 参数临时设置
    DEVICE_DISCOVERY    = 0xFF     // 设备发现
};


// 解析状态枚举
enum class ParseFrameState {
    START,              // 起始状态，等待帧头
    FRAME_HEADER,       // 帧头部
    FRAME_DATA_LEN,     // 数据长度
    FRAME_DATA,         // 数据内容
    FRAME_CHECKSUM      // 校验和
};


// 协议帧头结构
#pragma pack(push, 1)  // 确保结构体紧凑排列
struct FrameHeader {
    uint8_t start[2];    // 起始码 0xA5 0x5A
    uint8_t srcAddr;     // 源地址
    uint8_t destAddr;    // 目标地址
    CommandCode command; // 命令码
    uint8_t lengthLow;   // 参数长度低字节
    uint8_t lengthHigh;  // 参数长度高字节
};


// 完整协议帧结构
struct ProtocolFrame {
    FrameHeader header;
    std::vector<uint8_t> data;  // 可变长数据
    uint8_t checksum;           // 校验和
    uint64_t timestamp;          // 时间戳
};
#pragma pack(pop)


// 基础命令解析类
class CommandParse {
public:
    virtual ~CommandParse() = default;
    
    // 解析数据的纯虚函数
    virtual bool parse(const ProtocolFrame& frame) = 0;
    
    // 打印解析结果
    virtual void print() const = 0;
    
    // 获取命令码（用于识别具体的命令类型）
    virtual uint8_t getCommandCode() const = 0;
};


// 0xa8 命令解析类
class TargetInfoParse_0xA8 : public CommandParse {
public:
    struct TargetInfo {
        uint32_t type;
        float x_axes;
        float y_axes;
        float z_axes;
        float rangIdx;
        float speed;
        float peakVal;
        float dopplerIdx;
        float aoa_snr;
        int64_t timestamp;
    };

    // 公共接口
    bool parse(const ProtocolFrame& frame) override;
    void print() const override;
    uint8_t getCommandCode() const override { return 0xA8; }

    // 数据访问方法
    const std::vector<TargetInfo>& getTargets() const { return targets; }
    size_t getTargetCount() const { return targets.size(); }
    
    // 如果需要修改数据的接口
    void clearTargets() { targets.clear(); }
    void addTarget(const TargetInfo& target) { targets.push_back(target); }

private:
    std::vector<TargetInfo> targets;
    size_t targetCount = 0;
};


// 定义可能的返回类型组合
using CommandParseResult = std::variant<
    std::monostate,  // 表示无数据
    std::vector<TargetInfoParse_0xA8::TargetInfo>
>;


// 协议类
class Protocol {
public:
    // 协议相关常量
    static constexpr uint8_t FRAME_HEADER_0 = 0xA5;
    static constexpr uint8_t FRAME_HEADER_1 = 0x5A;
    static constexpr size_t HEADER_SIZE = sizeof(FrameHeader);
    static constexpr size_t MIN_FRAME_SIZE = HEADER_SIZE + 1;  // 头部 + 校验和
    static constexpr bool autoTimestamp = false;

    // 计算校验和
    static uint8_t calculateChecksum(const uint8_t* data, size_t len);

    // 小端数转uint_64_t
    static uint64_t littleEndian2Uint64(const uint8_t* data);
    
    // 打包协议帧
    static std::vector<uint8_t> packFrame(uint8_t srcAddr, uint8_t destAddr, 
                                        CommandCode command, const std::vector<uint8_t>& data = {});
    
    // 解析数据，将原始数据转为协议帧格式
    static bool parseFrame(const uint8_t* data, size_t len, ProtocolFrame& frame);

    // 命令解析函数
    static bool parseCommandData(const ProtocolFrame& frame, CommandParseResult& result);
};

// TcpClient 类
class TcpClient {
private:
    std::string serverAddress;
    int port;
#ifdef _WIN32
    SOCKET sockfd;  // Windows下使用SOCKET类型
#else
    int sockfd;     // Unix系统下使用int类型
#endif
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
    
    #ifdef _WIN32
        SOCKET getSocket() const { return sockfd; }
    #else
        int getSocket() const { return sockfd; }
    #endif
};

// TCP原始数据操作类
class TcpCommandHandler {
private:
    std::shared_ptr<TcpClient> tcpClient;   // 智能指针管理TcpClient对象
    ParseFrameState parseState;  // 解析状态
    std::vector<uint8_t> parseBuffer;  // 解析缓冲区
    size_t parseBufCount;  // 解析缓冲区计数
    size_t expectedDataLen;  // 预期数据长度

    // 添加接收缓冲区
    static constexpr size_t RECV_BUFFER_SIZE = 81920;
    std::vector<uint8_t> recvBuffer;
    size_t dataStart = 0;  // 有效数据起始位置
    size_t dataSize = 0;   // 有效数据大小

    std::atomic<bool> isRunning{false};  // 用于控制异步接收线程

    // 文件保存相关成员
    std::ofstream dataFile;
    bool isRecording = false;
    std::string saveFilePath;
    int64_t recordStartTime;  // 添加记录开始时间

public:
    explicit TcpCommandHandler(const std::string& address = "192.168.10.117", int port = 50000)
        : tcpClient(std::make_shared<TcpClient>(address, port))
        , parseState(ParseFrameState::START)
        , parseBufCount(0)
        , expectedDataLen(0) {
            recvBuffer.resize(RECV_BUFFER_SIZE);
        }

    ~TcpCommandHandler() {
        stopAsyncReceive();
    }

    // 连接管理
    bool connect() { return tcpClient->connect(); }
    bool disconnect() { return tcpClient->disconnect(); }
    bool isConnected() const { return tcpClient->isConnected(); }
    
    // 发送数据
    bool sendFrame(uint8_t srcAddr, uint8_t destAddr, CommandCode command, 
                  const std::vector<uint8_t>& data = {});
    bool quickSendCommand(CommandCode command, const std::vector<uint8_t>& data = {}) {
        // 使用默认地址发送命令
        return sendFrame(0x10, 0x90, command, data);
    }

    // 接收数据
    bool receiveFrame(ProtocolFrame& frame);    // 接收数据，并解析为协议帧
    bool receiveAndParseFrame(CommandParseResult& result);  // 接收数据，并解析为命令结果
    
    // 添加异步接收回调支持
    using FrameHandler = std::function<void(const ProtocolFrame&)>;
    void setFrameHandler(FrameHandler handler) {
        frameHandler = std::move(handler);
    }

    // 新增：启动异步接收
    bool startAsyncReceive();
    void stopAsyncReceive();

    // 超时设置
    bool setReceiveTimeout(int milliseconds) {
        return tcpClient->setReceiveTimeout(milliseconds);
    }

    // 添加数据记录控制方法
    bool startRecording(const std::string& filePath, std::string type);
    void stopRecording();
    bool isDataRecording() const { return isRecording; }
    void saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, std::string type);


private:
    // 重置解析状态
    void resetParser();

    bool parseFrame(const std::vector<uint8_t>& data, ProtocolFrame& frame);
    FrameHandler frameHandler;  // 回调函数

    // 新增：异步接收线程函数
    void asyncReceiveLoop();
};
