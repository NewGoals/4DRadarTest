#include "TcpCommandHandler.hpp"

//==============================================================================
// 0xa8 命令解析类实现
//==============================================================================
bool TargetInfoParse_0xA8::parse(const ProtocolFrame &frame)
{
    size_t length = frame.data.size();
    const uint8_t *data = frame.data.data();
    if (length < 4)
    {
        std::cerr << "【错误】A8命令数据长度不足以包含目标个数" << std::endl;
        return false;
    }

    // 分模式统计
    auto mode = getRadarMode();
    auto mode_parse_start = std::chrono::high_resolution_clock::now();

    // 分不同的输出模式进行解析
    if (getRadarMode() == 0)
    {
        // 模式0解析耗时统计
        // auto trace_parse_start = std::chrono::high_resolution_clock::now();

        targetCount = data[0];
        // std::cout << "targetCount: " << targetCount << std::endl;
        traces.clear();
        traces.reserve(targetCount);

        // 解析每个目标的数据
        const uint8_t *ptr = data + 1; // 跳过目标个数，其长度为1个字节
        for (size_t i = 0; i < targetCount; ++i)
        {
            TargetTrace trace;
            size_t offset = 0;

            memcpy(&trace.ID, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.type, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.x_speed, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.y_speed, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.z_speed, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.x_axes, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.y_axes, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.z_axes, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.length, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.azimuth_angle, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.elevation_angle, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.SNR, ptr + offset, 4);
            offset += 4;
            memcpy(&trace.Peak_energy, ptr + offset, 4);

            // 输出点迹详细信息
            // std::cout << "\n=== 目标 " << i + 1 << "/" << targetCount << " ===" << std::endl;
            // std::cout << std::hex; // 以16进制显示整数字段
            // std::cout << "ID: 0x" << trace.ID << "\t\t类型: 0x" << trace.type << std::endl;
            // std::cout << std::dec; // 切回十进制显示
            // std::cout << "坐标 (X/Y/Z): "
            //           << trace.x_axes << "m, "
            //           << trace.y_axes << "m, "
            //           << trace.z_axes << "m\n";
            // std::cout << "速度 (X/Y/Z): "
            //           << trace.x_speed << "m/s, "
            //           << trace.y_speed << "m/s, "
            //           << trace.z_speed << "m/s\n";
            // std::cout << "长度: " << trace.length << "m\n";
            // std::cout << "方位角: " << trace.azimuth_angle << "°\t仰角: "
            //           << trace.elevation_angle << "°\n";
            // std::cout << "信噪比: " << trace.SNR << "dB\t峰值能量: "
            //           << trace.Peak_energy << std::endl;

            // // 输出原始字节内容（可选）
            // std::cout << "\n原始数据 (Hex): ";
            // const uint8_t *raw = reinterpret_cast<const uint8_t *>(ptr);
            // for (int j = 0; j < 68; ++j)
            // { // 68字节完整数据
            //     std::cout << std::hex << std::setw(2) << std::setfill('0')
            //               << static_cast<int>(raw[j]) << " ";
            // }
            // std::cout << std::dec << "\n"
            //           << std::endl;

            // 还余16个字节，跳过保留字节

            // 轨迹过滤操作
            if(trace.x_axes > 1000 || trace.y_axes > 1000 || trace.z_axes > 1000 || trace.type == 0){
                ptr += 68;
                continue;
            }
            traces.push_back(trace);
            ptr += 68; // 移动到下一个目标
        }

        // // 模式0总耗时
        // auto trace_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        //     std::chrono::high_resolution_clock::now() - trace_parse_start).count();
        // std::cout << "[Perf] Mode0 parse: " << trace_duration << "μs (" 
        //           << targetCount << " targets)" << std::endl;
    }
    else if (getRadarMode() == 2)
    {
        // 解析目标个数 N (小端存储)
        targetCount = (data[3] << 8) | data[2]; // 只使用低两字节
        // std::cout << "targetCount: " <<int(targetCount) << std::endl;
        // std::cout << "first point type: " << int(data[4]) << std::endl;

        // 验证数据长度是否正确 (M = 1 + N*36)
        size_t expectedLen = 4 + targetCount * 36; // 4字节目标个数 + N个目标数据

        // 进行参数长度验证
        if (length != expectedLen)
        {
            std::cout << std::hex;
            std::cout << "【错误】A8命令数据长度不匹配 预期:" << expectedLen
                      << " 实际:" << length << std::endl;
            std::cout << std::dec;

            return false;
        }
        // 清空之前的数据
        targets.clear();
        targets.reserve(targetCount);

        // 解析每个目标的数据
        const uint8_t *ptr = data + 4; // 跳过目标个数字段
        if (Protocol::autoTimestamp)
        {
            for (size_t i = 0; i < targetCount; ++i)
            {
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
                ptr += 36; // 移动到下一个目标数据
            }
        }
        else
        {
            for (size_t i = 0; i < targetCount; ++i)
            {
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
                ptr += 36; // 移动到下一个目标数据
            }
        }
    }

    return true;
}

void TargetInfoParse_0xA8::print() const
{
    std::cout << "【A8命令解析结果】" << std::endl;
    std::cout << "目标总数: " << targetCount << std::endl;

    for (size_t i = 0; i < targets.size(); ++i)
    {
        const auto &target = targets[i];
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
uint8_t Protocol::calculateChecksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0;
    for (size_t i = 2; i < len; i++)
    {
        sum += data[i];
    }

    return static_cast<uint8_t>(sum & 0xFF); // 取最后一个字节（相当于对256求余）
    // return 0xff;
}

uint64_t Protocol::littleEndian2Uint64(const uint8_t *data)
{
    uint64_t result = 0;
    for (int i = 0; i < 8; ++i)
    {
        result |= static_cast<uint64_t>(data[i]) << (i * 8);
    }
    return result;
}

std::vector<uint8_t> Protocol::packFrame(uint8_t srcAddr, uint8_t destAddr,
                                         CommandCode command, const std::vector<uint8_t> &data)
{
    std::vector<uint8_t> frame;
    frame.reserve(HEADER_SIZE + data.size() + 1); // 预分配空间

    // 添加帧头
    frame.push_back(FRAME_HEADER_0);                // 起始码1
    frame.push_back(FRAME_HEADER_1);                // 起始码2
    frame.push_back(srcAddr);                       // 源地址
    frame.push_back(destAddr);                      // 目标地址
    frame.push_back(static_cast<uint8_t>(command)); // 命令码

    // 添加数据长度（小端序）
    uint16_t len = static_cast<uint16_t>(data.size());
    frame.push_back(static_cast<uint8_t>(len & 0xFF));        // 低字节
    frame.push_back(static_cast<uint8_t>((len >> 8) & 0xFF)); // 高字节

    // 添加数据
    frame.insert(frame.end(), data.begin(), data.end());

    // 计算并添加校验和
    uint8_t checksum = calculateChecksum(frame.data(), frame.size());
    frame.push_back(checksum);

    return frame;
}

bool Protocol::parseFrame(const uint8_t *data, size_t len, ProtocolFrame &frame)
{
    if (len < MIN_FRAME_SIZE)
    {
        std::cout << "【解析失败】数据长度不足" << std::endl;
        return false;
    }

    // 检查帧头
    if (data[0] != FRAME_HEADER_0 || data[1] != FRAME_HEADER_1)
    {
        std::cout << "【解析失败】帧头错误" << std::endl;
        return false;
    }

    // 复制头部数据
    memcpy(&frame.header, data, HEADER_SIZE);

    // const uint8_t* header_ptr = reinterpret_cast<const uint8_t*>(&frame.header);
    // for (size_t i = 0; i < sizeof(frame.header); ++i) {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0')
    //               << static_cast<int>(header_ptr[i]) << " ";
    // }
    // std::cout << std::dec << std::endl;

    // 获取帧信息中包含的数据长度,这里和参数长度保持一致，为2字节
    uint16_t dataLen = frame.header.lengthLow | (frame.header.lengthHigh << 8);
    size_t targetCount = (data[10] << 8) | data[9]; // 只使用低两字节
    // 获取实际数据长度，这里必须要使用2个字节以上的范围，即不可使用uint16，使得可以表示2000个点，70000以上大小的数据
    size_t expect_datalen = 4 + targetCount * 36;
    // std::cout << "expect_targetlen: " << targetCount << std::endl;
    // std::cout << "expect_datalen: " << expect_datalen << std::endl;

    // 这里没必要验证参数长度是否与计算长度相等，因为只要数量超过了2字节的范围，验证总是失效的
    // if (len != HEADER_SIZE + dataLen + 1) {
    //     std::cout << "【解析失败】数据长度不匹配" << std::endl;
    //     return false;
    // }

    // 复制数据
    // frame.data.assign(data + HEADER_SIZE, data + HEADER_SIZE + dataLen);
    frame.data.assign(data + HEADER_SIZE, data + HEADER_SIZE + expect_datalen);

    // 获取校验和
    frame.checksum = data[len - 1];

    // 如果校验和为0xff, 则不进行校验
    if (frame.checksum == 0xff)
    {
        // std::cout << "校验成功" << std::endl;
        return true;
    }

    uint8_t calculatedChecksum = calculateChecksum(data, len - 1);
    if (calculatedChecksum != frame.checksum)
    {
        std::cout << "【校验失败】校验和不相等，帧解析失败" << std::endl;
        // std::cout << "【校验详情】计算值: 0x" << std::hex << static_cast<int>(calculatedChecksum)
        //           << " 接收值: 0x" << static_cast<int>(frame.checksum) << std::endl;
    }
    return calculatedChecksum == frame.checksum;
}

bool Protocol::parseFrame_0(const uint8_t *data, size_t len, ProtocolFrame &frame)
{
    if (len < MIN_FRAME_SIZE)
    {
        std::cout << "【解析失败】数据长度不足" << std::endl;
        return false;
    }
    // 检查帧头
    if (data[0] != FRAME_HEADER_0 || data[1] != FRAME_HEADER_1)
    {
        std::cout << "【解析失败】帧头错误" << std::endl;
        return false;
    }

    // 复制头部数据，其中多出来的一个字节为无效数据
    memcpy(&frame.header, data, HEADER_SIZE);

    uint16_t dataLen = frame.header.lengthLow | (frame.header.lengthHigh << 8);
    size_t targetCount = data[7];
    uint16_t expect_datalen = 1 + targetCount * 68;
    frame.data.assign(data + HEADER_SIZE, data + HEADER_SIZE+ expect_datalen);

    // 不进行校验，直接返回
    return true;
}

bool Protocol::parseCommandData(const ProtocolFrame &frame, CommandParseResult &result)
{
    std::unique_ptr<CommandParse> cmdParser;

    switch (frame.header.command)
    {
    case CommandCode::TARGET_INFO:
    {
        auto cmdParser = std::make_unique<TargetInfoParse_0xA8>();
        if (cmdParser && cmdParser->parse(frame))
        {
            // cmdParser->print();
            if (cmdParser->getRadarMode() == 0)
            {
                result = cmdParser->getTraces();
                return true;
            }
            if (cmdParser->getRadarMode() == 2)
            {
                result = cmdParser->getTargets();
                return true;
            }
            return false;
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
TcpClient::TcpClient(const std::string &address, int port)
    : serverAddress(address), port(port), sockfd(-1), connected(false)
{
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "WSAStartup failed" << std::endl;
        throw std::runtime_error("WSAStartup failed");
    }
#endif
}

TcpClient::~TcpClient()
{
    if (connected)
    {
        disconnect();
    }
#ifdef _WIN32
    WSACleanup();
#endif
}

// bool TcpClient::connect()
// {
//     sockfd = socket(AF_INET, SOCK_STREAM, 0);
//     if (sockfd < 0)
//     {
//         return false;
//     }

//     struct sockaddr_in serverAddr;
//     memset(&serverAddr, 0, sizeof(serverAddr));
//     serverAddr.sin_family = AF_INET;
//     serverAddr.sin_port = htons(port);

//     if (inet_pton(AF_INET, serverAddress.c_str(), &serverAddr.sin_addr) <= 0)
//     {
// #ifdef _WIN32
//         closesocket(sockfd);
// #else
//         close(sockfd);
// #endif
//         return false;
//     }

//     if (::connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
//     {
// #ifdef _WIN32
//         int error = WSAGetLastError();
//         std::cerr << "Connect failed with error: " << error << std::endl;
//         closesocket(sockfd);
// #else
//         std::cerr << "Connect failed: " << strerror(errno) << std::endl;
//         close(sockfd);
// #endif
//         return false;
//     }

//     connected = true;
//     return true;
// }

// TCP connect DEBUG测试
bool TcpClient::connect()
{
    if (connected) {
        disconnect();
    }

    // 打印连接信息
    std::cout << "尝试连接到服务器: " << serverAddress << " 端口: " << port << std::endl;

    // 创建socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "创建socket失败" << std::endl;
        return false;
    }

    // 设置连接超时
#ifdef _WIN32
    // Windows平台设置超时
    DWORD timeout = 10000; // 10秒
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout));
#else
    // Linux平台设置超时
    struct timeval timeout;
    timeout.tv_sec = 10; // 10秒
    timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(timeout));
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout, sizeof(timeout));
#endif

    // 设置非阻塞模式
#ifdef _WIN32
    u_long mode = 1; // 非阻塞模式
    if (ioctlsocket(sockfd, FIONBIO, &mode) != 0)
    {
        std::cerr << "设置非阻塞模式失败: " << WSAGetLastError() << std::endl;
        closesocket(sockfd);
        return false;
    }
#else
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        std::cerr << "设置非阻塞模式失败: " << strerror(errno) << std::endl;
        close(sockfd);
        return false;
    }
#endif

    // 准备服务器地址
    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    // 将IP地址字符串转换为网络地址
    std::cout << "转换IP地址: " << serverAddress << std::endl;
    if (inet_pton(AF_INET, serverAddress.c_str(), &serverAddr.sin_addr) <= 0)
    {
        std::cerr << "IP地址无效: " << serverAddress << std::endl;
#ifdef _WIN32
        closesocket(sockfd);
#else
        close(sockfd);
#endif
        return false;
    }

    // 尝试连接
    std::cout << "开始连接..." << std::endl;
    int connectResult = ::connect(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

#ifdef _WIN32
    if (connectResult < 0)
    {
        int error = WSAGetLastError();
        if (error != WSAEWOULDBLOCK && error != WSAEINPROGRESS)
        {
            std::cerr << "连接失败，错误码: " << error << std::endl;
            closesocket(sockfd);
            return false;
        }
    }

    // 等待连接完成或超时
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sockfd, &writefds);

    // 设置select超时
    struct timeval selectTimeout;
    selectTimeout.tv_sec = 10; // 10秒超时
    selectTimeout.tv_usec = 0;

    // 等待连接完成
    if (select(sockfd + 1, NULL, &writefds, NULL, &selectTimeout) <= 0)
    {
        std::cerr << "连接超时或select错误: " << WSAGetLastError() << std::endl;
        closesocket(sockfd);
        return false;
    }

    // 检查连接是否成功
    int optVal;
    int optLen = sizeof(optVal);
    if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, (char *)&optVal, &optLen) < 0 || optVal != 0)
    {
        std::cerr << "连接错误: " << (optVal != 0 ? optVal : WSAGetLastError()) << std::endl;
        closesocket(sockfd);
        return false;
    }

    // 恢复阻塞模式
    mode = 0; // 阻塞模式
    if (ioctlsocket(sockfd, FIONBIO, &mode) != 0)
    {
        std::cerr << "恢复阻塞模式失败: " << WSAGetLastError() << std::endl;
        closesocket(sockfd);
        return false;
    }
#else
    if (connectResult < 0)
    {
        if (errno != EINPROGRESS)
        {
            std::cerr << "连接失败: " << strerror(errno) << std::endl;
            close(sockfd);
            return false;
        }
    }

    // 等待连接完成或超时
    fd_set writefds;
    FD_ZERO(&writefds);
    FD_SET(sockfd, &writefds);

    // 设置select超时
    struct timeval selectTimeout;
    selectTimeout.tv_sec = 10; // 10秒超时
    selectTimeout.tv_usec = 0;

    // 等待连接完成
    if (select(sockfd + 1, NULL, &writefds, NULL, &selectTimeout) <= 0)
    {
        std::cerr << "连接超时或select错误: " << strerror(errno) << std::endl;
        close(sockfd);
        return false;
    }

    // 检查连接是否成功
    int optVal;
    socklen_t optLen = sizeof(optVal);
    if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &optVal, &optLen) < 0 || optVal != 0)
    {
        std::cerr << "连接错误: " << (optVal != 0 ? strerror(optVal) : strerror(errno)) << std::endl;
        close(sockfd);
        return false;
    }

    // 恢复阻塞模式
    flags = fcntl(sockfd, F_GETFL, 0);
    if (fcntl(sockfd, F_SETFL, flags & ~O_NONBLOCK) == -1)
    {
        std::cerr << "恢复阻塞模式失败: " << strerror(errno) << std::endl;
        close(sockfd);
        return false;
    }
#endif

    std::cout << "连接成功!" << std::endl;
    connected = true;
    return true;
}

bool TcpClient::disconnect()
{
    if (!connected)
    {
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

ssize_t TcpClient::read(uint8_t *buffer, size_t size)
{
    if (!connected)
    {
        return -1;
    }
#ifdef _WIN32
    return recv(sockfd, (char *)buffer, static_cast<int>(size), 0);
#else
    return ::read(sockfd, buffer, size);
#endif
}

ssize_t TcpClient::write(const uint8_t *data, size_t size)
{
    if (!connected)
    {
        return -1;
    }
#ifdef _WIN32
    return send(sockfd, (char *)data, static_cast<int>(size), 0);
#else
    return ::write(sockfd, data, size);
#endif
}

//==============================================================================
// TcpCommandHandler 类实现
//==============================================================================
bool TcpCommandHandler::sendFrame(uint8_t type, uint8_t address,
                                  CommandCode command, const std::vector<uint8_t> &data)
{
    auto frameData = Protocol::packFrame(type, address, command, data);
    ssize_t sent = tcpClient->write(frameData.data(), frameData.size());
    return sent == frameData.size();
}

bool TcpCommandHandler::receiveFrame(ProtocolFrame &frame)
{
    const size_t THRESHOLD = RECV_BUFFER_SIZE * 2 / 4;
    const size_t MAX_BUFFER_BACKLOG = 20000; // 缓冲区积压阈值，超过此值将触发跳帧机制
    const size_t ESTIMATED_FRAME_SIZE = 5000; // 估计的平均帧大小
    static auto lastSkipTime = std::chrono::steady_clock::now(); // 上次跳帧的时间
    static const auto SKIP_INTERVAL = std::chrono::milliseconds(500); // 跳帧间隔时间控制

    // std::cout << "dataStart: " << dataStart << ", dataSize: " << dataSize << std::endl;
    // std::cout << "recvBuffer Size: " << recvBuffer.size() << std::endl;
    // 数据移动（缓冲区管理优化）
    if (dataStart + dataSize > THRESHOLD)
    {
        if (dataSize > 0)
        {
            std::memmove(recvBuffer.data(), recvBuffer.data() + dataStart, dataSize);
        }
        dataStart = 0;
        // std::cout << "【缓冲区重置】移动数据后, dataStart: 0, dataSize: " << dataSize << std::endl;
    }

    // 接收新数据
    size_t freeSpace = RECV_BUFFER_SIZE - (dataStart + dataSize);
    if (freeSpace > 0)
    {
        ssize_t received = tcpClient->read(recvBuffer.data() + dataStart + dataSize, freeSpace);
        if (received > 0)
        {
            dataSize += received;
            // std::cout << "【接收数据】新接收: " << std::dec << received
            //           << " bytes, 当前缓冲区大小: " << dataSize << std::endl;
        }
        else if (received == 0)
        {
            std::cerr << "【警告】对端已关闭连接。" << std::endl;
            return false;
        }
        else if (received < 0 && (errno != EAGAIN && errno != EWOULDBLOCK))
        {
            char errMsg[256];
#ifdef _WIN32
            strerror_s(errMsg, sizeof(errMsg), errno);
#else
            strncpy(errMsg, strerror(errno), sizeof(errMsg) - 1);
            errMsg[sizeof(errMsg) - 1] = '\0';
#endif
            std::cerr << "【错误】读取数据失败: " << errMsg << std::endl;
            return false;
        }
    }

    // // 添加调试信息
    // std::cout << "【数据内容】";
    // for (size_t i = dataStart; i < dataStart + std::min<size_t>(dataSize, 16); ++i)
    // {
    //     std::cout << std::hex << std::setw(2) << std::setfill('0')
    //               << static_cast<int>(recvBuffer[i]) << std::dec << " ";
    // }
    // std::cout << std::dec << std::endl;

    // 实现跳帧机制 - 当缓冲区积压过多数据时跳过旧帧
    // auto currentTime = std::chrono::steady_clock::now();
    // if (dataSize > MAX_BUFFER_BACKLOG && 
    //     currentTime - lastSkipTime > SKIP_INTERVAL)
    // {
    //     lastSkipTime = currentTime;
    //     size_t originalDataSize = dataSize;
    //     size_t framesToSkip = (dataSize - MAX_BUFFER_BACKLOG / 2) / ESTIMATED_FRAME_SIZE;
        
    //     // 实现跳帧 - 寻找新的帧头
    //     bool foundNewHeader = false;
    //     size_t newPos = dataStart;
    //     size_t endPos = dataStart + dataSize - Protocol::HEADER_SIZE; // 保留至少一个完整帧头的空间
        
    //     for (size_t i = dataStart; i < endPos; i++)
    //     {
    //         if (recvBuffer[i] == Protocol::FRAME_HEADER_0 && 
    //             recvBuffer[i + 1] == Protocol::FRAME_HEADER_1)
    //         {
    //             newPos = i;
    //             foundNewHeader = true;
    //             break;
    //         }
    //     }
        
    //     if (foundNewHeader)
    //     {
    //         size_t skippedBytes = newPos - dataStart;
    //         dataStart = newPos;
    //         dataSize -= skippedBytes;
    //         std::cout << "【跳帧】跳过 " << skippedBytes << " 字节旧数据，约 " 
    //                   << skippedBytes / ESTIMATED_FRAME_SIZE << " 帧" << std::endl;
    //     }
    // }

    // 预分配解析缓冲区空间以提高性能
    if (parseBuffer.capacity() < 10000) {
        parseBuffer.reserve(10000);
    }

    // 数据解析
    while (dataSize > 0)
    {
        uint8_t byte = recvBuffer[dataStart];
        switch (parseState)
        {
        case ParseFrameState::START:
            if (byte == Protocol::FRAME_HEADER_0)
            {
                parseBuffer.clear();
                parseBuffer.push_back(byte);
                parseBufCount = 1;
                parseState = ParseFrameState::FRAME_HEADER;
                // 记录帧处理开始时间
                frameStartTime = std::chrono::high_resolution_clock::now();  // 新增
            }
            dataStart++;
            dataSize--;
            break;

        case ParseFrameState::FRAME_HEADER:
            if (parseBufCount == 1 && byte != Protocol::FRAME_HEADER_1)
            {
                resetParser();
                break;
            }
            parseBuffer.push_back(byte);
            parseBufCount++;
            dataStart++;
            dataSize--;

            // 此处需要继续验证data部分的前四个字节，防止因为点数过多而使得参数长度大小大于2个字节，导致获取点数不全
            // if (parseBufCount == Protocol::HEADER_SIZE) {
            if (m_modeType == 2)
            {
                if (parseBufCount == Protocol::HEADER_SIZE + 4)
                {
                    std::vector<uint8_t> header_data(parseBuffer.begin(), parseBuffer.end() - 4);
                    FrameHeader *header = reinterpret_cast<FrameHeader *>(header_data.data());
                    size_t paramLength = header->lengthLow | (header->lengthHigh << 8);

                    std::vector<uint8_t> targetCount_data(parseBuffer.end() - 4, parseBuffer.end());
                    size_t targetCount = (targetCount_data[3] << 8) | targetCount_data[2]; // 只使用低两字节

                    expectedDataLen = 4 + targetCount * 36;

                    // std::cout << std::hex;
                    // std::cout << "【帧头解析完成】预检验参数长度 预期:" << paramLength
                    //           << " 实际:" << expectedDataLen << std::endl;
                    // std::cout << std::dec;

                    // std::cout << "【帧头解析完成】预期总长度: " << (Protocol::HEADER_SIZE + expectedDataLen + 1) << std::endl;
                    // std::cout << "【帧头解析完成】预期点云个数: " << targetCount << std::endl;

                    parseState = ParseFrameState::FRAME_DATA;
                }
                break;
            }
            else if (m_modeType == 0)
            {
                if (parseBufCount == Protocol::HEADER_SIZE + 1)
                {
                    std::vector<uint8_t> header_data(parseBuffer.begin(), parseBuffer.end() - 1);
                    // 此时不分高低字节，当前的header取到命令码后一位，需要进行补全操作再赋值
                    FrameHeader *header = reinterpret_cast<FrameHeader *>(header_data.data());

                    size_t paramLength = header->lengthLow | (header->lengthHigh << 8);;
                    std::vector<uint8_t> targetCount_data(parseBuffer.end() - 1, parseBuffer.end());
                    size_t targetCount = targetCount_data[0];
                    expectedDataLen = 1 + targetCount * 68;

                    // std::cout << std::hex;
                    // std::cout << "【帧头解析完成】预检验参数长度 预期:" << expectedDataLen
                    //           << " 实际:" << expectedDataLen << std::endl;
                    // std::cout << std::dec;

                    // std::cout << "【帧头解析完成】预期总长度: " << (Protocol::HEADER_SIZE - 1 + expectedDataLen + 1) << std::endl;
                    // std::cout << "【帧头解析完成】预期目标个数: " << targetCount << std::endl;

                    parseState = ParseFrameState::FRAME_DATA;
                }
                break;
            }
            else
            {
                std::cout << "TcpCommandHandler::receiveFrame 雷达输出数据类型未进行设置! " << std::endl;
                return false;
            }

        case ParseFrameState::FRAME_DATA:
        {
            // std::cout << "【数据统计】当前帧数据段已解析: " << parseBuffer.size() - Protocol::HEADER_SIZE
            //           << "/" << expectedDataLen << " 字节"
            //           << ", 缓冲区剩余可用: " << dataSize
            //           << " 字节"<< std::endl;

            // 剩余需要解析的数据量
            size_t remainingData;
            if (m_modeType == 2)
            {
                remainingData = expectedDataLen - 4;
            }
            else
            {
                remainingData = expectedDataLen - 1;
            }

            if (dataSize < remainingData + 1)
            { // +1 是因为校验和占一个字节
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

        case ParseFrameState::FRAME_CHECKSUM:
        {
            if (dataSize < 1)
            {
                return false;
            }

            parseBuffer.push_back(recvBuffer[dataStart]);
            dataStart++;
            dataSize--;

            bool parseFrame_success = false;
            if (m_modeType == 0)
            {

                parseFrame_success = Protocol::parseFrame_0(parseBuffer.data(), parseBuffer.size(), frame);
            }
            else
            {
                parseFrame_success = Protocol::parseFrame(parseBuffer.data(), parseBuffer.size(), frame);
            }
            if (parseFrame_success)
            {
                // std::cout << "【校验成功】帧解析完成" << std::endl;

                // 计算并输出处理时间
                auto endTime = std::chrono::high_resolution_clock::now();  // 新增
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - frameStartTime).count();  // 新增
                // std::cout << "【帧处理时间】" << duration << " μs" << std::endl;  // 新增
                resetParser();
                if(dataSize < 2000){
                   return true; 
                }
                else continue;
                
            }
            else
            {
                std::cout << "【校验失败】帧解析失败" << std::endl;
            }

            resetParser();
            continue; // 继续寻找下一个可能的帧
        }
        }
    }
    return false;
}

// 接收数据，并解析为命令结果
bool TcpCommandHandler::receiveAndParseFrame(CommandParseResult &result)
{
    ProtocolFrame frame;
    if (!receiveFrame(frame))
    {
        return false;
    }
    return Protocol::parseCommandData(frame, result);
}

// startAsyncReceive 和 stopAsyncReceive 方法实现
bool TcpCommandHandler::startAsyncReceive()
{
    if (isRunning)
        return false;

    isRunning = true;
    std::thread([this]()
                { this->asyncReceiveLoop(); })
        .detach();

    return true;
}

void TcpCommandHandler::stopAsyncReceive()
{
    isRunning = false;
}

bool TcpCommandHandler::parseFrame(const std::vector<uint8_t> &data, ProtocolFrame &frame)
{
    return Protocol::parseFrame(data.data(), data.size(), frame);
}

void TcpCommandHandler::resetParser()
{
    parseState = ParseFrameState::START;
    parseBuffer.clear();
    parseBufCount = 0;

    if (dataSize > 2)
    {
        // 在剩余数据中查找帧头
        for (size_t i = dataStart; i < dataStart + dataSize - 1; ++i)
        {
            if (recvBuffer[i] == Protocol::FRAME_HEADER_0 &&
                recvBuffer[i + 1] == Protocol::FRAME_HEADER_1)
            {
                dataSize = (dataStart + dataSize) - i; // 修正计算方式
                dataStart = i;
                // std::cout << "resetParser: 剩余数据中找到帧头，修正计算方式" << std::endl;
                return;
            }
        }
        // 如果没找到帧头，保留最后两个字节，因为它们可能是下一个帧的开始
        if (dataSize > 2)
        {
            std::cout << "resetParser: 未找到帧头，保留最后两个字节" << std::endl;
            dataSize = 2;
            dataStart = dataStart + dataSize - 2;
        }
    }
}

// 异步接收线程函数
void TcpCommandHandler::asyncReceiveLoop()
{
    while (isRunning && isConnected())
    {
        ProtocolFrame frame;
        if (receiveFrame(frame) && frameHandler)
        {
            frameHandler(frame);
        }
    }
}

bool TcpCommandHandler::startRecording(const std::string &filePath, std::string type)
{
    if (isRecording)
        return false;
    recordStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
    if (type == "csv")
    {
        dataFile.open(filePath, std::ios::out);
        if (!dataFile.is_open())
            return false;
        dataFile << "timestamp,type,x,y,z,range,speed,peak,doppler,snr\n";
        saveFilePath = filePath;
    }
    else if (type == "bin")
    {
        dataFile.open(filePath, std::ios::out | std::ios::binary);
        if (!dataFile.is_open())
        {
            std::cout << "TcpCommandHandler::startRecording无法打开BIN文件" << std::endl;
            return false;
        }
    }

    isRecording = true;
    return true;
}

void TcpCommandHandler::stopRecording()
{
    if (isRecording)
    {
        dataFile.close();
        isRecording = false;
    }
}

void TcpCommandHandler::saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo> &targets, std::string type)
{
    if (!isRecording || !dataFile.is_open())
        return;

    // 计算耗时
    // auto start = std::chrono::high_resolution_clock::now();

    if (type == "csv")
    {
        // 使用buffer写入
        std::ostringstream buffer;
        for (const auto &target : targets)
        {
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
    else if (type == "bin")
    {
        // 预先计算总大小
        size_t totalSize = sizeof(uint32_t) + // 目标数量
                           targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo);

        // 写入目标数量
        uint32_t targetCount = static_cast<uint32_t>(targets.size());
        dataFile.write(reinterpret_cast<const char *>(&targetCount), sizeof(targetCount));

        // 一次性写入所有目标数据
        dataFile.write(reinterpret_cast<const char *>(targets.data()),
                       targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo));
    }

    // auto end = std::chrono::high_resolution_clock::now(); // 记录结束时间
    // std::chrono::duration<double, std::milli> elapsed = end - start; // 计算时间差，改为毫秒级

    // std::cout << "点云数量" << targets.size() << ", 写入耗时: " << elapsed.count() << " ms" << std::endl; // 输出写入耗时
}