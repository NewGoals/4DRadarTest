#include "SynchronizedCollector.hpp"

//==============================================================================
// VideoSource 实现
//==============================================================================
VideoSource::VideoSource(const std::string& videoPath, const std::string& name) 
    : sourceName(name) {
    auto reader = DataReaderFactory::createReader(ReaderType::VIDEO_STREAM, videoPath);
    this->reader = std::dynamic_pointer_cast<VideoStreamReader>(reader);
}

bool VideoSource::init() {
    return reader && reader->init();
}

bool VideoSource::capture(int64_t timestamp) {
    std::unique_lock<std::shared_mutex> lock(dataMutex);    // 独占锁

    if (!reader->grabFrame()) {
        return false;
    }

    auto data = reader->getData();
    if (!data) return false;

    lastFrame = std::dynamic_pointer_cast<ImageData>(data)->frame.clone();
    lastTimestamp = timestamp;  // 记录实际的采集时间戳，后续可能很少用到，该时间戳来自相机

    // std::cout << "getcurrent time: " << std::dynamic_pointer_cast<ImageData>(data)->timestamp << std::endl;
    // std::cout << "cap time: " << timestamp << std::endl;
    return true;
}

void VideoSource::stop() {
    // 清理资源
}

std::string VideoSource::getSourceName() const {
    return sourceName;
}

const cv::Mat& VideoSource::getLastFrame() const{
    std::shared_lock<std::shared_mutex> lock(dataMutex);    // 共享锁
    return lastFrame;
}


//==============================================================================
// RadarSource 实现
//==============================================================================    
RadarSource::RadarSource(const std::string& ip, int port, const std::string& name)
    : handler(std::make_unique<TcpCommandHandler>(ip, port))
    , sourceName(name) {}

bool RadarSource::init() {
    return handler->connect();
}

bool RadarSource::capture(int64_t timestamp) {
    std::unique_lock<std::shared_mutex> lock(dataMutex);    // 使用独占锁
    CommandParseResult result;
    if (!handler->receiveAndParseFrame(result)){
        // std::cout << "RadarSource::capture error: hanlder->receiveAndParse. " << std::endl;
        return false;
    }
    
    if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
        lastTargets = *targets;  // 保存数据到成员变量
        lastTimestamp = timestamp;
        return true;
    }
    return false;
}

void RadarSource::stop() {
    handler->disconnect();
}

std::string RadarSource::getSourceName() const {
    return sourceName;
}


void RadarSource::saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, const std::string& path, RadarFileReader::Format saveFormat) {
    if(handler){
        if(saveFormat == RadarFileReader::Format::CSV){
            handler->startRecording(path, "csv");
            handler->saveTargetData(targets, "csv");
            handler->stopRecording();
        }
        else if(saveFormat == RadarFileReader::Format::BIN){
            handler->startRecording(path, "bin");
            handler->saveTargetData(targets, "bin");
            handler->stopRecording();
        }
    }
}

std::vector<TargetInfoParse_0xA8::TargetInfo> RadarSource::getLastTargets() const {
    std::shared_lock<std::shared_mutex> lock(dataMutex);    // 共享锁
    return lastTargets;
}


//==============================================================================
// SynchronizedCollector 实现
//==============================================================================
void SynchronizedCollector::addSource(std::unique_ptr<DataSource> source, bool isMainSource) {
    if (isMainSource) {
        mainSource = std::move(source);
    } else {
        subSources.push_back(std::move(source));
        captureThreads.emplace_back();  // 为从源创建采集线程
    }
}

void SynchronizedCollector::start() {
    std::cout << "开始数据采集..." << std::endl;
    
    if (!mainSource) {
        throw std::runtime_error("未设置主数据源");
    }
    
    isRunning = true;
    startTime = std::chrono::steady_clock::now();
    
    // 启动主数据源线程
    captureThreads.emplace_back();
    captureThreads.back().thread = std::thread(&SynchronizedCollector::mainSourceLoop, this);
    
    // 启动从数据源线程
    for (size_t i = 0; i < subSources.size(); ++i) {
        captureThreads.emplace_back();
        captureThreads.back().thread = std::thread(&SynchronizedCollector::subSourceLoop, this, i);
    }

    // 启动保存线程， 同步线程
    if(saveConfig.saveRadar || saveConfig.saveCamera){
        // 创建基础保存目录
        saveConfig.baseDir = "sync_data_" + getCurrentTimeString();
        std::filesystem::create_directory(saveConfig.baseDir);
        
        std::cout << "创建数据保存目录: " << saveConfig.baseDir << std::endl;
        saveThread = std::thread(&SynchronizedCollector::saveThreadLoop, this);

        // 启动同步线程
        syncThread = std::thread(&SynchronizedCollector::syncThreadLoop, this);
    }
    else{
        std::cout << "未启用保存线程" << std::endl;
    }
    
    std::cout << "所有采集线程已启动" << std::endl;
}

void SynchronizedCollector::mainSourceLoop() {
    auto& thread = captureThreads[0];   // 主源线程
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        
        if (auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())) {
            if (radarSource->capture(timestamp)) {
                thread.frameCount++;
                thread.lastCaptureTime = timestamp;

                {
                    std::lock_guard<std::mutex> lock(thread.bufferMutex);
                    // 如果缓存已满，则删除最早的帧
                    if (thread.radarFrameBuffer.size() >= thread.MAX_BUFFER_SIZE) {
                        thread.radarFrameBuffer.pop_front();
                    }
                    // 将最新帧添加到缓存中
                    auto newFrame = std::make_unique<RadarFrame>(radarSource->getLastTargets(), timestamp);
                    thread.radarFrameBuffer.push_back(std::move(newFrame));
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SynchronizedCollector::subSourceLoop(size_t sourceIndex) {
    auto& source = subSources[sourceIndex];
    auto& thread = captureThreads[sourceIndex + 1];
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        
        if (auto* videoSource = dynamic_cast<VideoSource*>(source.get())) {
            if (videoSource->capture(timestamp)) {
                thread.frameCount++;
                thread.lastCaptureTime = timestamp;
                
                {
                    std::lock_guard<std::mutex> lock(thread.bufferMutex);
                    // 如果缓存已满，则删除最早的帧
                    if (thread.imageFrameBuffer.size() >= thread.MAX_BUFFER_SIZE) {
                        thread.imageFrameBuffer.pop_front();
                    }
                    // 将最新帧添加到缓存中
                    int delay_time = 600;   // 延迟设置为600ms
                    auto newFrame = std::make_unique<ImageFrame>(videoSource->getLastFrame(), timestamp - 600);
                    thread.imageFrameBuffer.push_back(std::move(newFrame));
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SynchronizedCollector::saveThreadLoop(){
    // 新建保存文件夹
    if(saveConfig.saveRadar){
        std::string radarPath = saveConfig.baseDir + "/" + mainSource->getSourceName();
        std::filesystem::create_directory(radarPath);
    }
    if(saveConfig.saveCamera){
        for(const auto& source : subSources){
            std::string cameraPath = saveConfig.baseDir + "/" + source->getSourceName();
            std::filesystem::create_directory(cameraPath);
        }
    }

    while (isRunning) {
        SaveTask task;      // 从队列中取出一个保存任务
        {
            std::lock_guard<std::mutex> lock(saveMutex);
            if(saveQueue.size() > 10){
                std::cout << "当前保存任务队列长度: " << saveQueue.size() << std::endl;
            }

            if (saveQueue.empty()){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            task =std::move(saveQueue.front());
            saveQueue.pop();
        }

        // 保存雷达数据
        if(saveConfig.saveRadar){
            if(saveConfig.saveFormat == RadarFileReader::Format::CSV){
                std::string dataPath = saveConfig.baseDir + "/" + mainSource->getSourceName() + "/" + mainSource->getSourceName() + "_" + std::to_string(task.timestamp) + ".csv";
                // 这里需要调用handler的saveTargetData方法，保存task.radarData，而不能使用mainSource->saveData方法保存最近一帧数据
                if(auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())){
                    radarSource->saveTargetData(task.radarData, dataPath, RadarFileReader::Format::CSV);
                }
            }
            else if(saveConfig.saveFormat == RadarFileReader::Format::BIN){
                std::string dataPath = saveConfig.baseDir + "/" + mainSource->getSourceName() + "/" + mainSource->getSourceName() + "_" + std::to_string(task.timestamp) + ".bin";
                // 这里需要调用handler的saveTargetData方法，保存task.radarData，而不能使用mainSource->saveData方法保存最近一帧数据
                if(auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())){
                    radarSource->saveTargetData(task.radarData, dataPath, RadarFileReader::Format::BIN);
                }
            }
            
        }

        // 保存相机数据
        if(saveConfig.saveCamera){
            for(const auto& [sourceIndex, frame] : task.cameraFrames){
                if(!frame.empty()){
                    std::string framePath = saveConfig.baseDir + "/" + 
                                    subSources[sourceIndex]->getSourceName() + "/" +
                                    subSources[sourceIndex]->getSourceName() + "_" + 
                                    std::to_string(task.timestamp) + ".bmp";

                    // 计算耗时
                    // auto start = std::chrono::high_resolution_clock::now();
                    
                    cv::imwrite(framePath, frame);

                    // auto end = std::chrono::high_resolution_clock::now(); // 记录结束时间
                    // std::chrono::duration<double, std::milli> elapsed = end - start; // 计算时间差，改为毫秒级
                    
                    // std::cout << "图像写入耗时: " << elapsed.count() << " ms" << std::endl; // 输出写入耗时
                }
            }
        }
    }
}

void SynchronizedCollector::syncThreadLoop(){
    while (isRunning) {
        // 将雷达数据添加到保存任务中
        SaveTask task;
        
        // 取雷达数据时间
        int64_t radarTimestamp;
        {
            auto& mainThread = captureThreads[0];
            std::lock_guard<std::mutex> lock(mainThread.bufferMutex);
            if (mainThread.radarFrameBuffer.empty()) {
                // std::cout << "雷达队列缓存为空" << std::endl;
                continue;
            }
            radarTimestamp = mainThread.radarFrameBuffer.front()->timestamp;

            // 添加task
            task.radarData = mainThread.radarFrameBuffer.front()->targets;
            task.timestamp = radarTimestamp;
        }

        std::vector<std::pair<size_t, cv::Mat>> cameraFrames;
        {
            bool flag = false;
            bool radarEraseFlag = false;
            for(size_t i = 0; i < subSources.size(); ++i){
                cv::Mat newImage = findClosestFrame(captureThreads[i + 1], radarTimestamp, radarEraseFlag);
                if(!newImage.empty()){
                    task.cameraFrames.push_back(std::make_pair(i, std::move(newImage)));
                }
                else if(radarEraseFlag){
                    // 图像为空，且删除雷达数据
                    // 去掉队首
                    {
                        auto& mainThread = captureThreads[0];
                        std::lock_guard<std::mutex> lock(mainThread.bufferMutex);
                        mainThread.radarFrameBuffer.erase(mainThread.radarFrameBuffer.begin());       
                    }
                    // 如果存在某一帧匹配不了，即雷达数据过时，直接退出
                    flag = true;
                    break;
                }
                else{
                    // 等待图像更新
                    flag = true;
                    break;
                }
            }
            if(flag){
                // 不进行保存操作，而是等待图像更新
                continue;
            }
            else{
                // 表明同步正常，图像已添加进task，需要将雷达数据删除
                {
                    auto& mainThread = captureThreads[0];
                    std::lock_guard<std::mutex> lock(mainThread.bufferMutex);
                    mainThread.radarFrameBuffer.erase(mainThread.radarFrameBuffer.begin());       
                }
            }
        }

        {
            // 将保存任务添加到队列中
            std::lock_guard<std::mutex> lock(saveMutex);
            saveQueue.push(std::move(task)); 
        }
    }
}


cv::Mat SynchronizedCollector::findClosestFrame(CaptureThread& thread, int64_t timestamp, bool& radarEraseFlag) {
    std::lock_guard<std::mutex> lock(thread.bufferMutex);
    
    if (thread.imageFrameBuffer.empty()) {
        // std::cout << "当前帧缓存为空" << std::endl;
        return cv::Mat();
    }
    
    static const int64_t MAX_TIME_DIFF = 50;  // 50ms容忍度
    static const size_t MAX_BUFFER_RETAIN = 10;  // 保留最近的帧数
    size_t closestIndex = -1;
    int64_t minDiff = (std::numeric_limits<int64_t>::max)();    // 初始化最小差值为最大整数值
    int64_t prevDiff = (std::numeric_limits<int64_t>::max)();
    bool returnFlag = false;
    
    // 找到最接近的帧，不论是之前还是之后的
    // std::cout << "current image buffer size: " << thread.imageFrameBuffer.size() << std::endl;
    for (size_t i = 0; i < thread.imageFrameBuffer.size(); ++i) {
        auto& currentFrame = thread.imageFrameBuffer[i];
        int64_t diff = std::abs(currentFrame->timestamp - timestamp);

        // 如果差值开始变大，则停止搜索
        if (diff > prevDiff){
            returnFlag = true;
            break;   
        }

        if (diff < minDiff) {
            minDiff = diff;
            closestIndex = i;
        }
        prevDiff = diff;
    }

    // 如果没有找到合适的帧（证明此时退出原因为差值逐渐增加退出，且队列中的图像要比雷达数据更新，这里不做任何操作，直接返回等待雷达数据）
    if (minDiff > MAX_TIME_DIFF && returnFlag) {
        // 此时雷达数据已经过时，应该丢弃雷达数据，保存图像数据
        radarEraseFlag = true;
        return cv::Mat();
    }
    else if (minDiff < MAX_TIME_DIFF && returnFlag)
    {
        // 未遍历完退出，正常找到对应图像帧，删除之前的图像
        thread.imageFrameBuffer.erase(
            thread.imageFrameBuffer.begin(), 
            thread.imageFrameBuffer.begin() + closestIndex
        );
        if (!thread.imageFrameBuffer.empty()) {
            auto closestFramePtr = std::move(thread.imageFrameBuffer.front());
            if (std::abs(closestFramePtr->timestamp - timestamp) <= MAX_TIME_DIFF) {
                // 直接移动返回，避免克隆
                cv::Mat result = std::move(closestFramePtr->frame);
                thread.imageFrameBuffer.erase(thread.imageFrameBuffer.begin());
                return result;
            }
        }
    }
    else{
        // 这一步表示遍历完成，不管有没有找到对应的图像，都应该等待图像数据更新
        return cv::Mat();
    }
    
    // 一般不会到这里，但是还是调试输出
    std::cout << "获取到的帧存在问题，为空" << std::endl;
    return cv::Mat();
}

void SynchronizedCollector::stop() {
    isRunning = false;
    for (auto& ct : captureThreads) {
        if (ct.thread.joinable()) {
            ct.thread.join();
        }
    }
    for (auto& source : subSources) {
        source->stop();
    }
    if(saveThread.joinable()){
        saveThread.join();
    }
}

void SynchronizedCollector::printStats() {
    std::cout << "\n=== 采集状态报告 ===" << std::endl;
    
    // 打印主数据源状态
    if (mainSource) {
        auto mainThread = &captureThreads[0];
        std::cout << "主数据源 (" << mainSource->getSourceName() << "): "
                  << "总帧数: " << mainThread->frameCount
                  << ", 缓存帧数: " << mainThread->radarFrameBuffer.size()
                  << ", 最后采集时间: " << mainThread->lastCaptureTime << "ms"
                  << ", 缓存利用率: " << (mainThread->radarFrameBuffer.size() * 100.0 / mainThread->MAX_BUFFER_SIZE) << "%"
                  << std::endl;
        {
            std::lock_guard<std::mutex> lock(mainThread->bufferMutex);
            if(!mainThread->radarFrameBuffer.empty()){
               std::cout << "雷达队首：" << mainThread->radarFrameBuffer.front()->timestamp
                      << ", 雷达队尾：" << mainThread->radarFrameBuffer.back()->timestamp
                      << std::endl; 
            }
        }
    }
    
    // 打印从数据源状态
    for (size_t i = 0; i < subSources.size(); ++i) {
        auto& thread = captureThreads[i + 1]; // +1 因为第一个是主数据源
        std::cout << "从数据源 (" << subSources[i]->getSourceName() << "): "
                  << "总帧数: " << thread.frameCount
                  << ", 缓存帧数: " << thread.imageFrameBuffer.size()
                  << ", 最后采集时间: " << thread.lastCaptureTime << "ms"
                  << ", 缓存利用率: " << (thread.imageFrameBuffer.size() * 100.0 / thread.MAX_BUFFER_SIZE) << "%"
                  << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(thread.bufferMutex);
            if(!thread.imageFrameBuffer.empty()){
               std::cout << "图像队首：" << thread.imageFrameBuffer.front()->timestamp
                      << ", 图像队尾：" << thread.imageFrameBuffer.back()->timestamp
                      << std::endl; 
            }
        }
    }

    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    std::cout << "总运行时间: " << static_cast<int>(duration) << "秒" 
              << ", 雷达平均帧率: " << static_cast<int>((captureThreads[0].frameCount / (duration ? duration : 1))) << " fps"
              << std::endl;
}

int64_t SynchronizedCollector::getCurrentTimestamp() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now - startTime).count();
}

std::string SynchronizedCollector::getCurrentTimeString() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    
    std::tm timeinfo;
    localtime_s(&timeinfo, &time);
    
    std::stringstream ss;
    ss << std::put_time(&timeinfo, "%Y%m%d_%H%M%S");
    return ss.str();
}

void SynchronizedCollector::setSaveConfig(bool saveRadar, bool saveCamera, RadarFileReader::Format saveFormat) {
    saveConfig.saveRadar = saveRadar;
    saveConfig.saveCamera = saveCamera;
    saveConfig.saveFormat = saveFormat;
}

std::shared_ptr<RadarData> SynchronizedCollector::getMainSourceData() const {
    std::shared_ptr<RadarData> radarData = std::make_shared<RadarData>();
    if(auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())){
        for(const auto& target : radarSource->getLastTargets()){
            RadarPoint point;
            point.x = target.x_axes;
            point.y = target.y_axes;
            point.z = target.z_axes;
            point.rcs = target.peakVal;
            point.v_r = target.speed;
            radarData->points.push_back(point);
        }
        radarData->timestamp = radarSource->getLastTimestamp();
    }
    return radarData;
}

std::vector<std::pair<size_t, cv::Mat>> SynchronizedCollector::getSubSourceData() const{
    std::vector<std::pair<size_t, cv::Mat>> data;
    for(size_t i = 0; i < subSources.size(); ++i){
        if(auto* videoSource = dynamic_cast<VideoSource*>(subSources[i].get())){
            data.push_back(std::make_pair(i, videoSource->getLastFrame()));
        }
    }
    return data;
}