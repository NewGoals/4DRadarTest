#pragma once
#include "DataReader.hpp"
#include <cstdint>
#include <vector>
#include <memory>
#include <functional>
#include <variant>

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
};
#pragma pack(pop)


// �������������
class CommandParse {
public:
    virtual ~CommandParse() = default;
    
    // �������ݵĴ��麯��
    virtual bool parse(const uint8_t* data, size_t length) = 0;
    
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
    };

    // �����ӿ�
    bool parse(const uint8_t* data, size_t length) override;
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

    // ����У���
    static uint8_t calculateChecksum(const uint8_t* data, size_t len);
    
    // ���Э��֡
    static std::vector<uint8_t> packFrame(uint8_t srcAddr, uint8_t destAddr, 
                                        CommandCode command, const std::vector<uint8_t>& data = {});
    
    // �������ݣ���ԭʼ����תΪЭ��֡��ʽ
    static bool parseFrame(const uint8_t* data, size_t len, ProtocolFrame& frame);

    // �����������
    static bool parseCommandData(const ProtocolFrame& frame, CommandParseResult& result);
};

// TCPԭʼ���ݲ�����
class TcpCommandHandler {
private:
    TcpClient& tcpClient;        // ����TcpClient����   
    ParseFrameState parseState;  // ����״̬
    std::vector<uint8_t> parseBuffer;  // ����������
    size_t parseBufCount;  // ��������������
    size_t expectedDataLen;  // Ԥ�����ݳ���

    // ��ӽ��ջ�����
    static constexpr size_t RECV_BUFFER_SIZE = 40960;
    std::vector<uint8_t> recvBuffer;
    size_t dataStart = 0;  // ��Ч������ʼλ��
    size_t dataSize = 0;   // ��Ч���ݴ�С

public:
    explicit TcpCommandHandler(TcpClient& client) 
        : tcpClient(client)
        , parseState(ParseFrameState::START)
        , parseBufCount(0)
        , expectedDataLen(0) {
            recvBuffer.resize(RECV_BUFFER_SIZE);
        }
    
    // ��������
    bool sendFrame(uint8_t srcAddr, uint8_t destAddr, CommandCode command, 
                  const std::vector<uint8_t>& data = {});
    
    // ���ղ���������
    bool receiveFrame(ProtocolFrame& frame);

    // ����µı�ݷ���
    bool sendCommand(CommandCode command, const std::vector<uint8_t>& data = {}) {
        // ʹ��Ĭ�ϵ�ַ��������
        return sendFrame(0x10, 0x90, command, data);
    }
    
    // ����첽���ջص�֧��
    using FrameHandler = std::function<void(const ProtocolFrame&)>;
    void setFrameHandler(FrameHandler handler) {
        frameHandler = handler;
    }

private:
    // ���ý���״̬
    void resetParser();

    bool parseFrame(const std::vector<uint8_t>& data, ProtocolFrame& frame);
    FrameHandler frameHandler;  // �ص�����
};
