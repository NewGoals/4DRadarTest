#include "TcpCommandHandler.hpp"


//==============================================================================
// 0xa8 ���������ʵ��
//==============================================================================
bool TargetInfoParse_0xA8::parse(const ProtocolFrame& frame) {
    size_t length = frame.data.size();
    const uint8_t* data = frame.data.data();
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
    if(Protocol::autoTimestamp){
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
    }
    else{
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
            target.timestamp = frame.timestamp;

            targets.push_back(target);
            ptr += 36;  // �ƶ�����һ��Ŀ������
        }
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

uint64_t Protocol::littleEndian2Uint64(const uint8_t* data) {
    uint64_t result = 0;
    for (int i = 0; i < 8; ++i) {
        result |= static_cast<uint64_t>(data[i]) << (i * 8);
    }
    return result;
}

std::vector<uint8_t> Protocol::packFrame(uint8_t srcAddr, uint8_t destAddr, 
                                       CommandCode command, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame;
    frame.reserve(HEADER_SIZE + data.size() + 1);  // Ԥ����ռ�
    
    // ���֡ͷ
    frame.push_back(FRAME_HEADER_0);  // ��ʼ��1
    frame.push_back(FRAME_HEADER_1);  // ��ʼ��2
    frame.push_back(srcAddr);    // Դ��ַ
    frame.push_back(destAddr);   // Ŀ���ַ
    frame.push_back(static_cast<uint8_t>(command));  // ������
    
    // ������ݳ��ȣ�С����
    uint16_t len = static_cast<uint16_t>(data.size());
    frame.push_back(static_cast<uint8_t>(len & 0xFF));         // ���ֽ�
    frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));  // ���ֽ�
    
    // �������
    frame.insert(frame.end(), data.begin(), data.end());
    
    // ���㲢���У���
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
    if(!autoTimestamp){
        if (len != HEADER_SIZE + dataLen + 1) {
            // std::cout << "������ʧ�ܡ����ݳ��Ȳ�ƥ��(��ʱ���)" << std::endl;
            return false;
        }
    }
    else{
        if (len != HEADER_SIZE + dataLen + 1 + sizeof(uint64_t)){
            // std::cout << "������ʧ�ܡ����ݳ��Ȳ�ƥ��(��ʱ���)" << std::endl;
            // std::cout << "src_data_len: " << HEADER_SIZE + dataLen + 1 << std::endl;
            // std::cout << "real_len: " << len << std::endl;
            return false;
        }
    }
    
    
    // ��������
    frame.data.assign(data + HEADER_SIZE, data + HEADER_SIZE + dataLen);
    
    // ��ȡУ��ͣ���ȡʱ���
    if(!autoTimestamp){
        frame.checksum = data[len - 1];
    }
    else{
        frame.checksum = data[len - 1 - 8];
        uint8_t* littleEndiantTimestamp = new uint8_t[sizeof(uint64_t)];
        memcpy(littleEndiantTimestamp, data + len - sizeof(uint64_t), sizeof(uint64_t));
        frame.timestamp = littleEndian2Uint64(littleEndiantTimestamp);
        delete littleEndiantTimestamp;
    }
    
    // ���У���Ϊ0xff, �򲻽���У��
    if (frame.checksum == 0xff) {
        // std::cout << "У��ɹ�" << std::endl;
        return true;
    }

    uint8_t calculatedChecksum = calculateChecksum(data, len - 1);
    if (calculatedChecksum != frame.checksum) {
        std::cout << "֡����ʧ��" << std::endl;
        // std::cout << "��У�����顿����ֵ: 0x" << std::hex << static_cast<int>(calculatedChecksum) 
        //           << " ����ֵ: 0x" << static_cast<int>(frame.checksum) << std::endl;
    }
    return calculatedChecksum == frame.checksum;
}

bool Protocol::parseCommandData(const ProtocolFrame& frame, CommandParseResult& result){
    std::unique_ptr<CommandParse> cmdParser;

    switch (frame.header.command) {
        case CommandCode::TARGET_INFO:
        {
            auto cmdParser = std::make_unique<TargetInfoParse_0xA8>();
            if (cmdParser && cmdParser->parse(frame)) {
                // cmdParser->print();
                result = cmdParser->getTargets();
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
// TcpClient ��ʵ��
//==============================================================================
TcpClient::TcpClient(const std::string& address, int port)
    : serverAddress(address), port(port), sockfd(-1), connected(false) {
    #ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
    #endif
}

TcpClient::~TcpClient() {
    if (connected) {
        disconnect();
    }
    #ifdef _WIN32
        WSACleanup();
    #endif
}

bool TcpClient::connect() {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        return false;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    
    if (inet_pton(AF_INET, serverAddress.c_str(), &serverAddr.sin_addr) <= 0) {
        #ifdef _WIN32
            closesocket(sockfd);
        #else
            close(sockfd);
        #endif
        return false;
    }

    if (::connect(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        #ifdef _WIN32
            closesocket(sockfd);
        #else
            close(sockfd);
        #endif
        return false;
    }

    connected = true;
    return true;
}

bool TcpClient::disconnect() {
    if (!connected) {
        return true;
    }
    
    #ifdef _WIN32
        closesocket(sockfd);
    #else
        close(sockfd);
    #endif
    connected = false;
    sockfd = -1;
    return true;
}

ssize_t TcpClient::read(uint8_t* buffer, size_t size) {
    if (!connected) {
        return -1;
    }
    #ifdef _WIN32
        return recv(sockfd, (char*)buffer, static_cast<int>(size), 0);
    #else
        return ::read(sockfd, buffer, size);
    #endif
}

ssize_t TcpClient::write(const uint8_t* data, size_t size) {
    if (!connected) {
        return -1;
    }
    #ifdef _WIN32
        return send(sockfd, (char*)data, static_cast<int>(size), 0);
    #else
        return ::write(sockfd, data, size);
    #endif
}


//==============================================================================
// TcpCommandHandler ��ʵ��
//==============================================================================
bool TcpCommandHandler::sendFrame(uint8_t type, uint8_t address, 
                                CommandCode command, const std::vector<uint8_t>& data) {
    auto frameData = Protocol::packFrame(type, address, command, data);
    ssize_t sent = tcpClient->write(frameData.data(), frameData.size());
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
        // std::cout << "�����������á��ƶ����ݺ�, dataStart: 0, dataSize: " << dataSize << std::endl;
    }

    // ����������
    size_t freeSpace = RECV_BUFFER_SIZE - (dataStart + dataSize);
    if (freeSpace > 0) {
        ssize_t received = tcpClient->read(recvBuffer.data() + dataStart + dataSize, freeSpace);
        if (received > 0) {
            dataSize += received;
            // std::cout << "���������ݡ��½���: " << std::dec << received 
            //           << " bytes, ��ǰ��������С: " << dataSize << std::endl;
        } else if (received == 0) {
            // std::cerr << "�����桿�Զ��ѹر����ӡ�" << std::endl;
            return false;
        } else if (received < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            char errMsg[256];
#ifdef _WIN32
            strerror_s(errMsg, sizeof(errMsg), errno);
#else
            strncpy(errMsg, strerror(errno), sizeof(errMsg) - 1);
            errMsg[sizeof(errMsg) - 1] = '\0';
#endif
            // std::cerr << "�����󡿶�ȡ����ʧ��: " << errMsg << std::endl;
            return false;
        }
    }

    // // ��ӵ�����Ϣ
    // std::cout << "���������ݡ�";
    // for (size_t i = dataStart; i < dataStart + std::min<size_t>(dataSize, 16); ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
    //               << static_cast<int>(recvBuffer[i]) << std::dec << " ";
    // }
    // std::cout << std::dec << std::endl;

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
                    
                    // std::cout << "��֡ͷ������ɡ�Ԥ���ܳ���: " << (Protocol::HEADER_SIZE + expectedDataLen + 1) << std::endl;

                    parseState = ParseFrameState::FRAME_DATA;
                }
                break;

            case ParseFrameState::FRAME_DATA: {
                // std::cout << "������ͳ�ơ���ǰ֡���ݶ��ѽ���: " << parseBuffer.size() - Protocol::HEADER_SIZE 
                //           << "/" << expectedDataLen << " �ֽ�"
                //           << ", ������ʣ�����: " << dataSize 
                //           << " �ֽ�"<< std::endl;

                // ʣ����Ҫ������������
                size_t remainingData = expectedDataLen;

                if (dataSize < remainingData + 1) {  // +1 ����ΪУ���ռһ���ֽ�   
                    // std::cout << "������״̬�����ݲ��㣬�ȴ���������" << std::endl;
                    return false;
                }

                // std::cout << "���������顿��Ҫ�������ݶ�: " << remainingData 
                //           << " �ֽ� + 1 �ֽ�У���" << std::endl;

                // ֻ������Ҫ��������
                parseBuffer.insert(parseBuffer.end(), 
                                     recvBuffer.begin() + dataStart,
                                     recvBuffer.begin() + dataStart + remainingData);
                
                // std::cout << "�����ݿ����������� " << remainingData 
                //           << " �ֽ�, ��ǰ����֡�ѽ�����С: " << parseBuffer.size() << std::endl;
                
                dataStart += remainingData;
                dataSize -= remainingData;
                parseState = ParseFrameState::FRAME_CHECKSUM;
                continue;
            }

            case ParseFrameState::FRAME_CHECKSUM: {
                if (dataSize < 1) {
                    return false;
                }
                if(!Protocol::autoTimestamp){
                    parseBuffer.push_back(recvBuffer[dataStart]);
                    dataStart++;
                    dataSize--;    
                }
                else{
                    parseBuffer.insert(parseBuffer.end(), 
                                     recvBuffer.begin() + dataStart,
                                     recvBuffer.begin() + dataStart + 1 + 8);
                }
                            

                if (Protocol::parseFrame(parseBuffer.data(), parseBuffer.size(), frame)) {
                    // std::cout << "��У��ɹ���֡�������" << std::endl;
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

// �������ݣ�������Ϊ������
bool TcpCommandHandler::receiveAndParseFrame(CommandParseResult& result) {
    ProtocolFrame frame;
    if (!receiveFrame(frame)) {
        return false;
    }
    return Protocol::parseCommandData(frame, result);
}

// startAsyncReceive �� stopAsyncReceive ����ʵ��
bool TcpCommandHandler::startAsyncReceive() {
    if (isRunning) return false;
    
    isRunning = true;
    std::thread([this]() {
        this->asyncReceiveLoop();
    }).detach();
    
    return true;
}

void TcpCommandHandler::stopAsyncReceive() {
    isRunning = false;
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

// �첽�����̺߳���
void TcpCommandHandler::asyncReceiveLoop() {
    while (isRunning && isConnected()) {
        ProtocolFrame frame;
        if (receiveFrame(frame) && frameHandler) {
            frameHandler(frame);
        }
    }
}

bool TcpCommandHandler::startRecording(const std::string& filePath, std::string type) {
    if (isRecording) return false;
    recordStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if(type == "csv"){
        dataFile.open(filePath, std::ios::out);
        if (!dataFile.is_open()) return false;
        dataFile << "timestamp,type,x,y,z,range,speed,peak,doppler,snr\n";
        saveFilePath = filePath;
    }
    else if(type == "bin"){
        dataFile.open(filePath, std::ios::out | std::ios::binary); 
        if (!dataFile.is_open()){
            std::cout << "TcpCommandHandler::startRecording�޷���BIN�ļ�" << std::endl;
            return false; 
        }
    }
    
    isRecording = true;
    return true;
}

void TcpCommandHandler::stopRecording() {
    if (isRecording) {
        dataFile.close();
        isRecording = false;
    }
}

void TcpCommandHandler::saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, std::string type) {
    if (!isRecording || !dataFile.is_open()) return;
    
    // �����ʱ
    // auto start = std::chrono::high_resolution_clock::now();
    
    if(type == "csv"){
        // ʹ��bufferд��
        std::ostringstream buffer;
        for (const auto& target : targets) {
            buffer << recordStartTime << ","
                << target.type << ","
                << target.x_axes << ","
                << target.y_axes << ","
                << target.z_axes << ","
                << target.rangIdx << ","
                << target.speed << ","
                << target.peakVal << ","
                << target.dopplerIdx << ","
                << target.aoa_snr << "\n";
        }

        // ������������һ����д���ļ�
        dataFile << buffer.str();
        dataFile.flush();
    }
    else if(type == "bin"){
        // Ԥ�ȼ����ܴ�С
        size_t totalSize = sizeof(uint32_t) +  // Ŀ������
                        targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo);

        // д��Ŀ������
        uint32_t targetCount = static_cast<uint32_t>(targets.size());
        dataFile.write(reinterpret_cast<const char*>(&targetCount), sizeof(targetCount));

        // һ����д������Ŀ������
        dataFile.write(reinterpret_cast<const char*>(targets.data()), 
                    targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo));
    }
    

    // auto end = std::chrono::high_resolution_clock::now(); // ��¼����ʱ��
    // std::chrono::duration<double, std::milli> elapsed = end - start; // ����ʱ����Ϊ���뼶
    
    // std::cout << "��������" << targets.size() << ", д���ʱ: " << elapsed.count() << " ms" << std::endl; // ���д���ʱ
}