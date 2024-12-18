#include "TcpCommandHandler.hpp"


//==============================================================================
// 0xa8 命令解析类实现
//==============================================================================
bool TargetInfoParse_0xA8::parse(const ProtocolFrame& frame) {
    size_t length = frame.data.size();
    const uint8_t* data = frame.data.data();
    if (length < 4) {
        std::cerr << "【错误】A8命令数据长度不足以包含目标个数" << std::endl;
        return false;
    }

    // 解析目标个数 N (小端存储)
    targetCount = (data[3] << 8) | data[2];  // 只使用低两字节
    
    // 验证数据长度是否正确 (M = 1 + N*36)
    size_t expectedLen = 4 + targetCount * 36;  // 4字节目标个数 + N个目标数据
    if (length != expectedLen) {
        std::cerr << "【错误】A8命令数据长度不匹配 预期:" << expectedLen 
                  << " 实际:" << length << std::endl;
        return false;
    }

    // 清空之前的数据
    targets.clear();
    targets.reserve(targetCount);

    // 解析每个目标的数据
    const uint8_t* ptr = data + 4;  // 跳过目标个数字段
    if(Protocol::autoTimestamp){
        for (size_t i = 0; i < targetCount; ++i) {
            TargetInfo target;
            
            // 解析目标数据
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
            ptr += 36;  // 移动到下一个目标数据
        }
    }
    else{
        for (size_t i = 0; i < targetCount; ++i) {
            TargetInfo target;
            
            // 解析目标数据
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
            ptr += 36;  // 移动到下一个目标数据
        }
    }
    

    return true;
}

void TargetInfoParse_0xA8::print() const {
    std::cout << "【A8命令解析结果】" << std::endl;
    std::cout << "目标总数: " << targetCount << std::endl;
    
    for (size_t i = 0; i < targets.size(); ++i) {
        const auto& target = targets[i];
        std::cout << "【目标 " << i + 1 << "】" << std::endl
                  << "  类型: " << (target.type == 0 ? "静态" : "动态") << std::endl
                  << "  位置: (" << target.x_axes << ", "
                  << target.y_axes << ", "
                  << target.z_axes << ") m" << std::endl
                  << "  速度: " << target.speed << std::endl
                  << "  距离维Idx: " << target.rangIdx << std::endl
                  << "  速度维Idx: " << target.dopplerIdx << std::endl
                  << "  能量峰值: " << target.peakVal << std::endl
                  << "  信噪比: " << target.aoa_snr << std::endl;
    }
}

//==============================================================================
// 协议类实现
//==============================================================================
uint8_t Protocol::calculateChecksum(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 2; i < len; i++) {
        sum += data[i];
    }

    return static_cast<uint8_t>(sum & 0xFF);  // 取最后一个字节（相当于对256求余）
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
    frame.reserve(HEADER_SIZE + data.size() + 1);  // 预分配空间
    
    // 添加帧头
    frame.push_back(FRAME_HEADER_0);  // 起始码1
    frame.push_back(FRAME_HEADER_1);  // 起始码2
    frame.push_back(srcAddr);    // 源地址
    frame.push_back(destAddr);   // 目标地址
    frame.push_back(static_cast<uint8_t>(command));  // 命令码
    
    // 添加数据长度（小端序）
    uint16_t len = static_cast<uint16_t>(data.size());
    frame.push_back(static_cast<uint8_t>(len & 0xFF));         // 低字节
    frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));  // 高字节
    
    // 添加数据
    frame.insert(frame.end(), data.begin(), data.end());
    
    // 计算并添加校验和
    uint8_t checksum = calculateChecksum(frame.data(), frame.size());
    frame.push_back(checksum);
    
    return frame;
}

bool Protocol::parseFrame(const uint8_t* data, size_t len, ProtocolFrame& frame) {
    if (len < MIN_FRAME_SIZE) {
        // std::cout << "【解析失败】数据长度不足" << std::endl;
        return false;
    }
    
    // 检查帧头
    if (data[0] != FRAME_HEADER_0 || data[1] != FRAME_HEADER_1) {
        // std::cout << "【解析失败】帧头错误" << std::endl;
        return false;
    }
    
    // 复制头部数据
    memcpy(&frame.header, data, HEADER_SIZE);
    
    // 获取数据长度
    uint16_t dataLen = frame.header.lengthLow | (frame.header.lengthHigh << 8);
    
    // 检查总长度
    if(!autoTimestamp){
        if (len != HEADER_SIZE + dataLen + 1) {
            // std::cout << "【解析失败】数据长度不匹配(无时间戳)" << std::endl;
            return false;
        }
    }
    else{
        if (len != HEADER_SIZE + dataLen + 1 + sizeof(uint64_t)){
            // std::cout << "【解析失败】数据长度不匹配(带时间戳)" << std::endl;
            // std::cout << "src_data_len: " << HEADER_SIZE + dataLen + 1 << std::endl;
            // std::cout << "real_len: " << len << std::endl;
            return false;
        }
    }
    
    
    // 复制数据
    frame.data.assign(data + HEADER_SIZE, data + HEADER_SIZE + dataLen);
    
    // 获取校验和，获取时间戳
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
    
    // 如果校验和为0xff, 则不进行校验
    if (frame.checksum == 0xff) {
        // std::cout << "校验成功" << std::endl;
        return true;
    }

    uint8_t calculatedChecksum = calculateChecksum(data, len - 1);
    if (calculatedChecksum != frame.checksum) {
        std::cout << "帧解析失败" << std::endl;
        // std::cout << "【校验详情】计算值: 0x" << std::hex << static_cast<int>(calculatedChecksum) 
        //           << " 接收值: 0x" << static_cast<int>(frame.checksum) << std::endl;
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
            std::cerr << "未知的命令码: 0x" << std::hex 
                      << static_cast<int>(frame.header.command) << std::endl;
            return false;
    }
    return false;
}


//==============================================================================
// TcpClient 类实现
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
// TcpCommandHandler 类实现
//==============================================================================
bool TcpCommandHandler::sendFrame(uint8_t type, uint8_t address, 
                                CommandCode command, const std::vector<uint8_t>& data) {
    auto frameData = Protocol::packFrame(type, address, command, data);
    ssize_t sent = tcpClient->write(frameData.data(), frameData.size());
    return sent == frameData.size();
}

bool TcpCommandHandler::receiveFrame(ProtocolFrame& frame) {
    const size_t THRESHOLD = RECV_BUFFER_SIZE * 3 / 4;

    // 数据移动（缓冲区管理优化）
    if (dataStart + dataSize > THRESHOLD) {
        if (dataSize > 0) {
            std::memmove(recvBuffer.data(), recvBuffer.data() + dataStart, dataSize);
        }
        dataStart = 0;
        // std::cout << "【缓冲区重置】移动数据后, dataStart: 0, dataSize: " << dataSize << std::endl;
    }

    // 接收新数据
    size_t freeSpace = RECV_BUFFER_SIZE - (dataStart + dataSize);
    if (freeSpace > 0) {
        ssize_t received = tcpClient->read(recvBuffer.data() + dataStart + dataSize, freeSpace);
        if (received > 0) {
            dataSize += received;
            // std::cout << "【接收数据】新接收: " << std::dec << received 
            //           << " bytes, 当前缓冲区大小: " << dataSize << std::endl;
        } else if (received == 0) {
            // std::cerr << "【警告】对端已关闭连接。" << std::endl;
            return false;
        } else if (received < 0 && (errno != EAGAIN && errno != EWOULDBLOCK)) {
            char errMsg[256];
#ifdef _WIN32
            strerror_s(errMsg, sizeof(errMsg), errno);
#else
            strncpy(errMsg, strerror(errno), sizeof(errMsg) - 1);
            errMsg[sizeof(errMsg) - 1] = '\0';
#endif
            // std::cerr << "【错误】读取数据失败: " << errMsg << std::endl;
            return false;
        }
    }

    // // 添加调试信息
    // std::cout << "【数据内容】";
    // for (size_t i = dataStart; i < dataStart + std::min<size_t>(dataSize, 16); ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0') 
    //               << static_cast<int>(recvBuffer[i]) << std::dec << " ";
    // }
    // std::cout << std::dec << std::endl;

    // 数据解析
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
                    
                    // std::cout << "【帧头解析完成】预期总长度: " << (Protocol::HEADER_SIZE + expectedDataLen + 1) << std::endl;

                    parseState = ParseFrameState::FRAME_DATA;
                }
                break;

            case ParseFrameState::FRAME_DATA: {
                // std::cout << "【数据统计】当前帧数据段已解析: " << parseBuffer.size() - Protocol::HEADER_SIZE 
                //           << "/" << expectedDataLen << " 字节"
                //           << ", 缓冲区剩余可用: " << dataSize 
                //           << " 字节"<< std::endl;

                // 剩余需要解析的数据量
                size_t remainingData = expectedDataLen;

                if (dataSize < remainingData + 1) {  // +1 是因为校验和占一个字节   
                    // std::cout << "【解析状态】数据不足，等待更多数据" << std::endl;
                    return false;
                }

                // std::cout << "【计算详情】需要复制数据段: " << remainingData 
                //           << " 字节 + 1 字节校验和" << std::endl;

                // 只复制需要的数据量
                parseBuffer.insert(parseBuffer.end(), 
                                     recvBuffer.begin() + dataStart,
                                     recvBuffer.begin() + dataStart + remainingData);
                
                // std::cout << "【数据拷贝】复制了 " << remainingData 
                //           << " 字节, 当前解析帧已解析大小: " << parseBuffer.size() << std::endl;
                
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
                    // std::cout << "【校验成功】帧解析完成" << std::endl;
                    resetParser();
                    return true;
                }
                else {
                    std::cout << "【校验失败】帧解析失败" << std::endl;
                }

                resetParser();
                continue;  // 继续寻找下一个可能的帧
            }
        }
    }
    return false;
}

// 接收数据，并解析为命令结果
bool TcpCommandHandler::receiveAndParseFrame(CommandParseResult& result) {
    ProtocolFrame frame;
    if (!receiveFrame(frame)) {
        return false;
    }
    return Protocol::parseCommandData(frame, result);
}

// startAsyncReceive 和 stopAsyncReceive 方法实现
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
        // 在剩余数据中查找帧头
        for (size_t i = dataStart; i < dataStart + dataSize - 1; ++i) {
            if (recvBuffer[i] == Protocol::FRAME_HEADER_0 && 
                recvBuffer[i + 1] == Protocol::FRAME_HEADER_1) {
                dataSize = (dataStart + dataSize) - i;  // 修正计算方式
                dataStart = i;
                return;
            }
        }
        // 如果没找到帧头，保留最后两个字节，因为它们可能是下一个帧的开始
        if (dataSize > 2) {
            dataSize = 2;
            dataStart = dataStart + dataSize - 2;
        }
    }
}

// 异步接收线程函数
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
            std::cout << "TcpCommandHandler::startRecording无法打开BIN文件" << std::endl;
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
    
    // 计算耗时
    // auto start = std::chrono::high_resolution_clock::now();
    
    if(type == "csv"){
        // 使用buffer写入
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

        // 将缓冲区内容一次性写入文件
        dataFile << buffer.str();
        dataFile.flush();
    }
    else if(type == "bin"){
        // 预先计算总大小
        size_t totalSize = sizeof(uint32_t) +  // 目标数量
                        targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo);

        // 写入目标数量
        uint32_t targetCount = static_cast<uint32_t>(targets.size());
        dataFile.write(reinterpret_cast<const char*>(&targetCount), sizeof(targetCount));

        // 一次性写入所有目标数据
        dataFile.write(reinterpret_cast<const char*>(targets.data()), 
                    targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo));
    }
    

    // auto end = std::chrono::high_resolution_clock::now(); // 记录结束时间
    // std::chrono::duration<double, std::milli> elapsed = end - start; // 计算时间差，改为毫秒级
    
    // std::cout << "点云数量" << targets.size() << ", 写入耗时: " << elapsed.count() << " ms" << std::endl; // 输出写入耗时
}