#pragma once
#ifdef _WIN32
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #include <WinSock2.h>  // ������ windows.h ֮ǰ
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


// ������ö�ٶ���
enum class CommandCode : uint8_t {
    FACTORY_RESET       = 0x01,    // �ָ���������
    ADD_COORDINATE      = 0x03,    // �����������
    NETWORK_CONFIG      = 0x04,    // ������������
    BROADCAST_NETWORK   = 0x05,    // �㲥�������
    HEARTBEAT_CONFIG    = 0x09,    // ����ʱ������
    READ_RADAR_STATUS   = 0x0A,    // ��ȡ�״�״̬
    SYSTEM_TIME         = 0x24,    // ϵͳʱ������
    SAVE_PARAMS         = 0x88,    // �������
    HEARTBEAT           = 0xA4,    // ������
    TARGET_INFO         = 0xA8,    // Ŀ����Ϣ����
    FIRMWARE_UPDATE     = 0xCC,    // �̼�����
    FILE_TRANSFER       = 0xD7,    // �ļ��������
    FILE_TRANSFER_END   = 0xD8,    // �ļ��������
    RADAR_ANGLE_CORRECT = 0xD9,    // �״�Ƕ�У��
    GET_ANGLE_STATUS    = 0xDA,    // ��ȡ�״�Ƕ�У��״̬
    SYSTEM_RESTART      = 0xDB,    // ����ϵͳ
    READ_PARAMS         = 0xE0,    // ������ȡ
    SET_PARAMS          = 0xE1,    // ��������
    TEMP_SET_PARAMS     = 0xE3,    // ������ʱ����
    DEVICE_DISCOVERY    = 0xFF     // �豸����
};


// ����״̬ö��
enum class ParseFrameState {
    START,              // ��ʼ״̬���ȴ�֡ͷ
    FRAME_HEADER,       // ֡ͷ��
    FRAME_DATA_LEN,     // ���ݳ���
    FRAME_DATA,         // ��������
    FRAME_CHECKSUM      // У���
};


// Э��֡ͷ�ṹ
#pragma pack(push, 1)  // ȷ���ṹ���������
struct FrameHeader {
    uint8_t start[2];    // ��ʼ�� 0xA5 0x5A
    uint8_t srcAddr;     // Դ��ַ
    uint8_t destAddr;    // Ŀ���ַ
    CommandCode command; // ������
    uint8_t lengthLow;   // �������ȵ��ֽ�
    uint8_t lengthHigh;  // �������ȸ��ֽ�
};


// ����Э��֡�ṹ
struct ProtocolFrame {
    FrameHeader header;
    std::vector<uint8_t> data;  // �ɱ䳤����
    uint8_t checksum;           // У���
    uint64_t timestamp;          // ʱ���
};
#pragma pack(pop)


// �������������
class CommandParse {
public:
    virtual ~CommandParse() = default;
    
    // �������ݵĴ��麯��
    virtual bool parse(const ProtocolFrame& frame) = 0;
    
    // ��ӡ�������
    virtual void print() const = 0;
    
    // ��ȡ�����루����ʶ�������������ͣ�
    virtual uint8_t getCommandCode() const = 0;
};


// 0xa8 ���������
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

    // �����ӿ�
    bool parse(const ProtocolFrame& frame) override;
    void print() const override;
    uint8_t getCommandCode() const override { return 0xA8; }

    // ���ݷ��ʷ���
    const std::vector<TargetInfo>& getTargets() const { return targets; }
    size_t getTargetCount() const { return targets.size(); }
    
    // �����Ҫ�޸����ݵĽӿ�
    void clearTargets() { targets.clear(); }
    void addTarget(const TargetInfo& target) { targets.push_back(target); }

private:
    std::vector<TargetInfo> targets;
    size_t targetCount = 0;
};


// ������ܵķ����������
using CommandParseResult = std::variant<
    std::monostate,  // ��ʾ������
    std::vector<TargetInfoParse_0xA8::TargetInfo>
>;


// Э����
class Protocol {
public:
    // Э����س���
    static constexpr uint8_t FRAME_HEADER_0 = 0xA5;
    static constexpr uint8_t FRAME_HEADER_1 = 0x5A;
    static constexpr size_t HEADER_SIZE = sizeof(FrameHeader);
    static constexpr size_t MIN_FRAME_SIZE = HEADER_SIZE + 1;  // ͷ�� + У���
    static constexpr bool autoTimestamp = false;

    // ����У���
    static uint8_t calculateChecksum(const uint8_t* data, size_t len);

    // С����תuint_64_t
    static uint64_t littleEndian2Uint64(const uint8_t* data);
    
    // ���Э��֡
    static std::vector<uint8_t> packFrame(uint8_t srcAddr, uint8_t destAddr, 
                                        CommandCode command, const std::vector<uint8_t>& data = {});
    
    // �������ݣ���ԭʼ����תΪЭ��֡��ʽ
    static bool parseFrame(const uint8_t* data, size_t len, ProtocolFrame& frame);

    // �����������
    static bool parseCommandData(const ProtocolFrame& frame, CommandParseResult& result);
};

// TcpClient ��
class TcpClient {
private:
    std::string serverAddress;
    int port;
#ifdef _WIN32
    SOCKET sockfd;  // Windows��ʹ��SOCKET����
#else
    int sockfd;     // Unixϵͳ��ʹ��int����
#endif
    bool connected;

public:
    TcpClient(const std::string& address = "192.168.10.117", int port = 50000);
    ~TcpClient();
    
    bool connect();
    bool disconnect();
    bool isConnected() const { return connected; }
    
    // ��д����
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

// TCPԭʼ���ݲ�����
class TcpCommandHandler {
private:
    std::shared_ptr<TcpClient> tcpClient;   // ����ָ�����TcpClient����
    ParseFrameState parseState;  // ����״̬
    std::vector<uint8_t> parseBuffer;  // ����������
    size_t parseBufCount;  // ��������������
    size_t expectedDataLen;  // Ԥ�����ݳ���

    // ��ӽ��ջ�����
    static constexpr size_t RECV_BUFFER_SIZE = 81920;
    std::vector<uint8_t> recvBuffer;
    size_t dataStart = 0;  // ��Ч������ʼλ��
    size_t dataSize = 0;   // ��Ч���ݴ�С

    std::atomic<bool> isRunning{false};  // ���ڿ����첽�����߳�

    // �ļ�������س�Ա
    std::ofstream dataFile;
    bool isRecording = false;
    std::string saveFilePath;
    int64_t recordStartTime;  // ��Ӽ�¼��ʼʱ��

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

    // ���ӹ���
    bool connect() { return tcpClient->connect(); }
    bool disconnect() { return tcpClient->disconnect(); }
    bool isConnected() const { return tcpClient->isConnected(); }
    
    // ��������
    bool sendFrame(uint8_t srcAddr, uint8_t destAddr, CommandCode command, 
                  const std::vector<uint8_t>& data = {});
    bool quickSendCommand(CommandCode command, const std::vector<uint8_t>& data = {}) {
        // ʹ��Ĭ�ϵ�ַ��������
        return sendFrame(0x10, 0x90, command, data);
    }

    // ��������
    bool receiveFrame(ProtocolFrame& frame);    // �������ݣ�������ΪЭ��֡
    bool receiveAndParseFrame(CommandParseResult& result);  // �������ݣ�������Ϊ������
    
    // ����첽���ջص�֧��
    using FrameHandler = std::function<void(const ProtocolFrame&)>;
    void setFrameHandler(FrameHandler handler) {
        frameHandler = std::move(handler);
    }

    // �����������첽����
    bool startAsyncReceive();
    void stopAsyncReceive();

    // ��ʱ����
    bool setReceiveTimeout(int milliseconds) {
        return tcpClient->setReceiveTimeout(milliseconds);
    }

    // ������ݼ�¼���Ʒ���
    bool startRecording(const std::string& filePath, std::string type);
    void stopRecording();
    bool isDataRecording() const { return isRecording; }
    void saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, std::string type);


private:
    // ���ý���״̬
    void resetParser();

    bool parseFrame(const std::vector<uint8_t>& data, ProtocolFrame& frame);
    FrameHandler frameHandler;  // �ص�����

    // �������첽�����̺߳���
    void asyncReceiveLoop();
};
