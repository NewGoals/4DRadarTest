#include <iomanip>
#include "TcpCommandHandler.hpp"


//==============================================================================
// 0xa8 ���������ʵ��
//==============================================================================
bool TargetInfoParse_0xA8::parse(const uint8_t* data, size_t length) {
    if (length < 4) {
        std::cerr << "������A8�������ݳ��Ȳ����԰���Ŀ�����" << std::endl;
        return false;
    }

    // ����Ŀ����� N (С�˴洢)
    targetCount = (data[3] << 8) | data[2];  // ֻʹ�õ����ֽ�
    
    // ��֤���ݳ����Ƿ���ȷ (M = 1 + N*36)
    size_t expectedLen = 4 + targetCount * 36;  // 4�ֽ�Ŀ����� + N��Ŀ������
    if (length != expectedLen) {
        std::cerr << "������A8�������ݳ��Ȳ�ƥ�� Ԥ��:" << expectedLen 
                  << " ʵ��:" << length << std::endl;
        return false;
    }

    // ���֮ǰ������
    targets.clear();
    targets.reserve(targetCount);

    // ����ÿ��Ŀ�������
    const uint8_t* ptr = data + 4;  // ����Ŀ������ֶ�
    for (size_t i = 0; i < targetCount; ++i) {
        TargetInfo target;
        
        // ����Ŀ������
        memcpy(&target.type, ptr, 4);
        memcpy(&target.x_axes, ptr + 4, 4);
        memcpy(&target.y_axes, ptr + 8, 4);
        memcpy(&target.z_axes, ptr + 12, 4);
        memcpy(&target.rangIdx, ptr + 16, 4);
        memcpy(&target.speed, ptr + 20, 4);
        memcpy(&target.peakVal, ptr + 24, 4);
        memcpy(&target.dopplerIdx, ptr + 28, 4);
        memcpy(&target.aoa_snr, ptr + 32, 4);

        targets.push_back(target);
        ptr += 36;  // �ƶ�����һ��Ŀ������
    }

    return true;
}

void TargetInfoParse_0xA8::print() const {
    std::cout << "��A8������������" << std::endl;
    std::cout << "Ŀ������: " << targetCount << std::endl;
    
    for (size_t i = 0; i < targets.size(); ++i) {
        const auto& target = targets[i];
        std::cout << "��Ŀ�� " << i + 1 << "��" << std::endl
                  << "  ����: " << (target.type == 0 ? "��̬" : "��̬") << std::endl
                  << "  λ��: (" << target.x_axes << ", "
                  << target.y_axes << ", "
                  << target.z_axes << ") m" << std::endl
                  << "  �ٶ�: " << target.speed << std::endl
                  << "  ����άIdx: " << target.rangIdx << std::endl
                  << "  �ٶ�άIdx: " << target.dopplerIdx << std::endl
                  << "  ������ֵ: " << target.peakVal << std::endl
                  << "  �����: " << target.aoa_snr << std::endl;
    }
}

//==============================================================================
// Э����ʵ��
//==============================================================================
uint8_t Protocol::calculateChecksum(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 2; i < len; i++) {
        sum += data[i];
    }

    return static_cast<uint8_t>(sum & 0xFF);  // ȡ���һ���ֽڣ��൱�ڶ�256���ࣩ
    // return 0xff;
}

std::vector<uint8_t> Protocol::packFrame(uint8_t srcAddr, uint8_t destAddr, 
                                       CommandCode command, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame;
    frame.reserve(HEADER_SIZE + data.size() + 1);  // Ԥ����ռ�
    
    // ����֡ͷ
    frame.push_back(FRAME_HEADER_0);  // ��ʼ��1
    frame.push_back(FRAME_HEADER_1);  // ��ʼ��2
    frame.push_back(srcAddr);    // Դ��ַ
    frame.push_back(destAddr);   // Ŀ���ַ
    frame.push_back(static_cast<uint8_t>(command));  // ������
    
    // �������ݳ��ȣ�С����
    uint16_t len = static_cast<uint16_t>(data.size());
    frame.push_back(static_cast<uint8_t>(len & 0xFF));         // ���ֽ�
    frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));  // ���ֽ�
    
    // ��������
    frame.insert(frame.end(), data.begin(), data.end());
    
    // ���㲢����У���
    uint8_t checksum = calculateChecksum(frame.data(), frame.size());
    frame.push_back(checksum);
    
    return frame;
}

bool Protocol::parseFrame(const uint8_t* data, size_t len, ProtocolFrame& frame) {
    if (len < MIN_FRAME_SIZE) {
        // std::cout << "������ʧ�ܡ����ݳ��Ȳ���" << std::endl;
        return false;
    }
    
    // ���֡ͷ
    if (data[0] != FRAME_HEADER_0 || data[1] != FRAME_HEADER_1) {
        // std::cout << "������ʧ�ܡ�֡ͷ����" << std::endl;
        return false;
    }
    
    // ����ͷ������
    memcpy(&frame.header, data, HEADER_SIZE);
    
    // ��ȡ���ݳ���
    uint16_t dataLen = frame.header.lengthLow | (frame.header.lengthHigh << 8);
    
    // ����ܳ���
    if (len != HEADER_SIZE + dataLen + 1) {
        // std::cout << "������ʧ�ܡ����ݳ��Ȳ�ƥ��" << std::endl;
        return false;
    }
    
    // ��������
    frame.data.assign(data + HEADER_SIZE, data + HEADER_SIZE + dataLen);
    
    // ��ȡУ���
    frame.checksum = data[len - 1];
    
    // ���У���Ϊ0xff, �򲻽���У��
    if (frame.checksum == 0xff) {
        return true;
    }

    uint8_t calculatedChecksum = calculateChecksum(data, len - 1);
    if (calculatedChecksum != frame.checksum) {
        std::cout << "��У�����顿����ֵ: 0x" << std::hex << static_cast<int>(calculatedChecksum) 
                  << " ����ֵ: 0x" << static_cast<int>(frame.checksum) << std::endl;
    }
    return calculatedChecksum == frame.checksum;
}

bool Protocol::parseCommandData(const ProtocolFrame& frame, CommandParseResult& result){
    std::unique_ptr<CommandParse> cmdParser;

    switch (frame.header.command) {
        case CommandCode::TARGET_INFO:
        {
            auto cmdParser = std::make_unique<TargetInfoParse_0xA8>();
            if (cmdParser && cmdParser->parse(frame.data.data(), frame.data.size())) {
                cmdParser->print();
                auto result = cmdParser->getTargets();
                return true;
            }
            break;
        }
            
        default:
            std::cerr << "δ֪��������: 0x" << std::hex 
                      << static_cast<int>(frame.header.command) << std::endl;
            return false;
    }
    return false;
}

//==============================================================================
// TcpCommandHandler ��ʵ��
//==============================================================================
bool TcpCommandHandler::sendFrame(uint8_t type, uint8_t address, 
                                CommandCode command, const std::vector<uint8_t>& data) {
    auto frameData = Protocol::packFrame(type, address, command, data);
    ssize_t sent = tcpClient.write(frameData.data(), frameData.size());
    return sent == frameData.size();
}

bool TcpCommandHandler::receiveFrame(ProtocolFrame& frame) {
    const size_t THRESHOLD = RECV_BUFFER_SIZE * 3 / 4;

    // �����ƶ��������������Ż���
    if (dataStart + dataSize > THRESHOLD) {
        if (dataSize > 0) {
            std::memmove(recvBuffer.data(), recvBuffer.data() + dataStart, dataSize);
        }
        dataStart = 0;
        std::cout << "�����������á��ƶ����ݺ�, dataStart: 0, dataSize: " << dataSize << std::endl;
    }

    // ����������
    size_t freeSpace = RECV_BUFFER_SIZE - (dataStart + dataSize);
    if (freeSpace > 0) {
        ssize_t received = tcpClient.read(recvBuffer.data() + dataStart + dataSize, freeSpace);
        if (received > 0) {
            dataSize += received;
            std::cout << "���������ݡ��½���: " << std::dec << received 
                      << " bytes, ��ǰ��������С: " << dataSize << std::endl;
        } else if (received == 0) {
            std::cerr << "�����桿�Զ��ѹر����ӡ�" << std::endl;
            return false;
        } else if (received < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            char errMsg[256];
#ifdef _WIN32
            strerror_s(errMsg, sizeof(errMsg), errno);
#else
            strncpy(errMsg, strerror(errno), sizeof(errMsg) - 1);
            errMsg[sizeof(errMsg) - 1] = '\0';
#endif
            std::cerr << "�����󡿶�ȡ����ʧ��: " << errMsg << std::endl;
            return false;
        }
    }

    // ���ӵ�����Ϣ
    std::cout << "���������ݡ�";
    for (size_t i = dataStart; i < dataStart + std::min<size_t>(dataSize, 16); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << static_cast<int>(recvBuffer[i]) << std::dec << " ";
    }
    std::cout << std::dec << std::endl;

    // ���ݽ���
    while (dataSize > 0) {
        uint8_t byte = recvBuffer[dataStart];
        switch (parseState) {
            case ParseFrameState::START:
                if (byte == Protocol::FRAME_HEADER_0) {
                    parseBuffer.clear();
                    parseBuffer.push_back(byte);
                    parseBufCount = 1;
                    parseState = ParseFrameState::FRAME_HEADER;
                }
                dataStart++;
                dataSize--;
                break;

            case ParseFrameState::FRAME_HEADER:
                if (parseBufCount == 1 && byte != Protocol::FRAME_HEADER_1) {
                    resetParser();
                    break;
                }
                parseBuffer.push_back(byte);
                parseBufCount++;
                dataStart++;
                dataSize--;

                if (parseBufCount == Protocol::HEADER_SIZE) {
                    FrameHeader* header = reinterpret_cast<FrameHeader*>(parseBuffer.data());
                    expectedDataLen = header->lengthLow | (header->lengthHigh << 8);
                    
                    std::cout << "��֡ͷ������ɡ�Ԥ���ܳ���: " << (Protocol::HEADER_SIZE + expectedDataLen + 1) << std::endl;

                    parseState = ParseFrameState::FRAME_DATA;
                }
                break;

            case ParseFrameState::FRAME_DATA: {
                std::cout << "������ͳ�ơ���ǰ֡���ݶ��ѽ���: " << parseBuffer.size() - Protocol::HEADER_SIZE 
                          << "/" << expectedDataLen << " �ֽ�"
                          << ", ������ʣ�����: " << dataSize 
                          << " �ֽ�"<< std::endl;

                // ʣ����Ҫ������������
                size_t remainingData = expectedDataLen;

                if (dataSize < remainingData + 1) {  // +1 ����ΪУ���ռһ���ֽ�   
                    std::cout << "������״̬�����ݲ��㣬�ȴ���������" << std::endl;
                    return false;
                }
                
                std::cout << "���������顿��Ҫ�������ݶ�: " << remainingData 
                          << " �ֽ� + 1 �ֽ�У���" << std::endl;

                // ֻ������Ҫ��������
                parseBuffer.insert(parseBuffer.end(), 
                                     recvBuffer.begin() + dataStart,
                                     recvBuffer.begin() + dataStart + remainingData);
                
                std::cout << "�����ݿ����������� " << remainingData 
                          << " �ֽ�, ��ǰ����֡�ѽ�����С: " << parseBuffer.size() << std::endl;
                
                dataStart += remainingData;
                dataSize -= remainingData;
                parseState = ParseFrameState::FRAME_CHECKSUM;
                continue;
            }

            case ParseFrameState::FRAME_CHECKSUM: {
                if (dataSize < 1) {
                    return false;
                }
                parseBuffer.push_back(recvBuffer[dataStart]);
                dataStart++;
                dataSize--;

                // �ж��������Ƿ�Ϊ0xa8, ������򲻽���У�� 
                

                if (Protocol::parseFrame(parseBuffer.data(), parseBuffer.size(), frame)) {
                    std::cout << "��У��ɹ���֡�������" << std::endl;
                    resetParser();
                    return true;
                }
                else {
                    std::cout << "��У��ʧ�ܡ�֡����ʧ��" << std::endl;
                }

                resetParser();
                continue;  // ����Ѱ����һ�����ܵ�֡
            }
        }
    }
    return false;
}

bool TcpCommandHandler::parseFrame(const std::vector<uint8_t>& data, ProtocolFrame& frame) {
    return Protocol::parseFrame(data.data(), data.size(), frame);
} 

void TcpCommandHandler::resetParser() {
    parseState = ParseFrameState::START;
    parseBuffer.clear();
    parseBufCount = 0;
    
    if (dataSize > 2) {
        // ��ʣ�������в���֡ͷ
        for (size_t i = dataStart; i < dataStart + dataSize - 1; ++i) {
            if (recvBuffer[i] == Protocol::FRAME_HEADER_0 && 
                recvBuffer[i + 1] == Protocol::FRAME_HEADER_1) {
                dataSize = (dataStart + dataSize) - i;  // �������㷽ʽ
                dataStart = i;
                return;
            }
        }
        // ���û�ҵ�֡ͷ��������������ֽڣ���Ϊ���ǿ�������һ��֡�Ŀ�ʼ
        if (dataSize > 2) {
            dataSize = 2;
            dataStart = dataStart + dataSize - 2;
        }
    }
}