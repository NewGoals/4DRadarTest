#include "SynchronizedCollector.hpp"

//==============================================================================
// VideoSource ʵ��
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
    std::unique_lock<std::shared_mutex> lock(dataMutex);    // ��ռ��

    if (!reader->grabFrame()) {
        return false;
    }

    auto data = reader->getData();
    if (!data) return false;

    lastFrame = std::dynamic_pointer_cast<ImageData>(data)->frame.clone();
    lastTimestamp = timestamp;  // ��¼ʵ�ʵĲɼ�ʱ������������ܺ����õ�����ʱ����������
    return true;
}

void VideoSource::stop() {
    // ������Դ
}

std::string VideoSource::getSourceName() const {
    return sourceName;
}

const cv::Mat& VideoSource::getLastFrame() const{
    std::shared_lock<std::shared_mutex> lock(dataMutex);    // ������
    return lastFrame;
}


//==============================================================================
// RadarSource ʵ��
//==============================================================================    
RadarSource::RadarSource(const std::string& ip, int port, const std::string& name)
    : handler(std::make_unique<TcpCommandHandler>(ip, port))
    , sourceName(name) {}

bool RadarSource::init() {
    return handler->connect();
}

bool RadarSource::capture(int64_t timestamp) {
    std::unique_lock<std::shared_mutex> lock(dataMutex);    // ʹ�ö�ռ��
    CommandParseResult result;
    if (!handler->receiveAndParseFrame(result)) return false;
    
    if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
        lastTargets = *targets;  // �������ݵ���Ա����
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

void RadarSource::saveData(const std::string& path) {
    handler->startRecording(path);
    handler->saveTargetData(lastTargets);
    handler->stopRecording();
}

void RadarSource::saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, const std::string& csv_path) {
    if(handler){
        handler->startRecording(csv_path);
        handler->saveTargetData(targets);
        handler->stopRecording();
    }
}

std::vector<TargetInfoParse_0xA8::TargetInfo> RadarSource::getLastTargets() const {
    std::shared_lock<std::shared_mutex> lock(dataMutex);    // ������
    return lastTargets;
}


//==============================================================================
// SynchronizedCollector ʵ��
//==============================================================================
void SynchronizedCollector::addSource(std::unique_ptr<DataSource> source, bool isMainSource) {
    if (isMainSource) {
        mainSource = std::move(source);
    } else {
        subSources.push_back(std::move(source));
        captureThreads.emplace_back();  // Ϊ��Դ�����ɼ��߳�
    }
}

void SynchronizedCollector::start() {
    std::cout << "��ʼ���ݲɼ�..." << std::endl;
    
    if (!mainSource) {
        throw std::runtime_error("δ����������Դ");
    }
    
    isRunning = true;
    startTime = std::chrono::steady_clock::now();
    
    // ����������Դ�߳�
    captureThreads.emplace_back();
    captureThreads.back().thread = std::thread(&SynchronizedCollector::mainSourceLoop, this);
    
    // ����������Դ�߳�
    for (size_t i = 0; i < subSources.size(); ++i) {
        captureThreads.emplace_back();
        captureThreads.back().thread = std::thread(&SynchronizedCollector::subSourceLoop, this, i);
    }

    // ���������߳�
    if(saveConfig.saveRadar || saveConfig.saveCamera){
        // ������������Ŀ¼
        saveConfig.baseDir = "sync_data_" + getCurrentTimeString();
        std::filesystem::create_directory(saveConfig.baseDir);
        
        std::cout << "�������ݱ���Ŀ¼: " << saveConfig.baseDir << std::endl;
        saveThread = std::thread(&SynchronizedCollector::saveThreadLoop, this);
    }
    else{
        std::cout << "δ���ñ����߳�" << std::endl;
    }
    
    std::cout << "���вɼ��߳�������" << std::endl;
}

void SynchronizedCollector::mainSourceLoop() {
    auto& thread = captureThreads[0];
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        
        if (auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())) {
            if (radarSource->capture(timestamp)) {
                thread.frameCount++;
                thread.lastCaptureTime = timestamp;

                // ���״�������ӵ�����������
                SaveTask task;
                task.timestamp = timestamp;
                task.radarData = radarSource->getLastTargets();

                // �����������ӵ�����������
                for(size_t i = 0; i < subSources.size(); ++i){
                    task.cameraFrames.push_back(std::make_pair(i, findClosestFrame(captureThreads[i + 1], timestamp)));
                }

                // ���������̲߳���ӣ��������һֱ���Ӷ��޷�����
                if(saveConfig.saveRadar || saveConfig.saveCamera)
                {   
                    // ������������ӵ�������
                    std::lock_guard<std::mutex> lock(saveMutex);
                    saveQueue.push(std::move(task));
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
                    // ���������������ɾ�������֡
                    if (thread.frameBuffer.size() >= thread.MAX_BUFFER_SIZE) {
                        thread.frameBuffer.pop_front();
                    }
                    // ������֡��ӵ�������
                    auto newFrame = std::make_unique<ImageFrame>(videoSource->getLastFrame(), timestamp);
                    thread.frameBuffer.push_back(std::move(newFrame));
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SynchronizedCollector::saveThreadLoop(){
    // �½������ļ���
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
        SaveTask task;      // �Ӷ�����ȡ��һ����������
        {
            std::lock_guard<std::mutex> lock(saveMutex);
            if(saveQueue.size() > 10){
                std::cout << "��ǰ����������г���: " << saveQueue.size() << std::endl;
            }

            if (saveQueue.empty()){
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            task =std::move(saveQueue.front());
            saveQueue.pop();
        }

        // �����״�����
        if(saveConfig.saveRadar){
            std::string dataPath = saveConfig.baseDir + "/" + mainSource->getSourceName() + "/" + mainSource->getSourceName() + "_" + std::to_string(task.timestamp) + ".csv";
            // ������Ҫ����handler��saveTargetData����������task.radarData��������ʹ��mainSource->saveData�����������һ֡����
            if(auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())){
                radarSource->saveTargetData(task.radarData, dataPath);
            }
        }

        // �����������
        if(saveConfig.saveCamera){
            for(const auto& [sourceIndex, frame] : task.cameraFrames){
                if(!frame.empty()){
                    std::string framePath = saveConfig.baseDir + "/" + 
                                    subSources[sourceIndex]->getSourceName() + "/" +
                                    subSources[sourceIndex]->getSourceName() + "_" + 
                                    std::to_string(task.timestamp) + ".jpg";
                    cv::imwrite(framePath, frame);
                }
            }
        }
    }
}


cv::Mat SynchronizedCollector::findClosestFrame(CaptureThread& thread, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(thread.bufferMutex);
    
    if (thread.frameBuffer.empty()) {
        std::cout << "��ǰ֡����Ϊ��" << std::endl;
        return cv::Mat();
    }
    
    static const int64_t MAX_TIME_DIFF = 50;  // 50ms���̶�
    static const size_t MAX_BUFFER_RETAIN = 10;  // ���������֡��
    size_t closestIndex = 0;
    int64_t minDiff = (std::numeric_limits<int64_t>::max)();    // ��ʼ����С��ֵΪ�������ֵ
    int64_t prevDiff = (std::numeric_limits<int64_t>::max)();
    
    // �ҵ���ӽ���֡��������֮ǰ����֮���
    for (size_t i = 0; i < thread.frameBuffer.size(); ++i) {
        auto& currentFrame = thread.frameBuffer[i];
        int64_t diff = std::abs(currentFrame->timestamp - timestamp);

        // �����ֵ��ʼ�����ֹͣ����
        if (diff > prevDiff) break;

        if (diff < minDiff) {
            minDiff = diff;
            closestIndex = i;
        }
        prevDiff = diff;
    }

    // ���û���ҵ����ʵ�֡
    if (minDiff > MAX_TIME_DIFF) {
        // ֻ��������ļ�֡��������ȫ��ջ�����
        if (thread.frameBuffer.size() > MAX_BUFFER_RETAIN) {
            thread.frameBuffer.erase(
                thread.frameBuffer.begin(), 
                thread.frameBuffer.begin() + (thread.frameBuffer.size() - MAX_BUFFER_RETAIN)
            );
        }
        return cv::Mat();
    }

    // ɾ�����ҵ���֮֡ǰ������֡
    thread.frameBuffer.erase(
        thread.frameBuffer.begin(), 
        thread.frameBuffer.begin() + closestIndex
    );

    // ����ҵ���֡�����̷�Χ�ڣ��򷵻�
    if (!thread.frameBuffer.empty()) {
        auto closestFramePtr = std::move(thread.frameBuffer.front());
        if (std::abs(closestFramePtr->timestamp - timestamp) <= MAX_TIME_DIFF) {
            // ֱ���ƶ����أ������¡
            cv::Mat result = std::move(closestFramePtr->frame);
            thread.frameBuffer.erase(thread.frameBuffer.begin());
            return result;
        }
    }
    
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
    std::cout << "\n=== �ɼ�״̬���� ===" << std::endl;
    
    // ��ӡ������Դ״̬
    if (mainSource) {
        auto mainThread = &captureThreads[0];
        std::cout << "������Դ (" << mainSource->getSourceName() << "): "
                  << "֡��: " << mainThread->frameCount 
                  << ", ���ɼ�ʱ��: " << mainThread->lastCaptureTime << "ms" << std::endl;
    }
    
    // ��ӡ������Դ״̬
    for (size_t i = 0; i < subSources.size(); ++i) {
        auto& thread = captureThreads[i + 1]; // +1 ��Ϊ��һ����������Դ
        std::cout << "������Դ (" << subSources[i]->getSourceName() << "): "
                  << "��֡��: " << thread.frameCount 
                  << ", ����֡��: " << thread.frameBuffer.size() 
                  << ", ���ɼ�ʱ��: " << thread.lastCaptureTime << "ms"
                  << ", ����������: " << (thread.frameBuffer.size() * 100.0 / thread.MAX_BUFFER_SIZE) << "%"
                  << std::endl;
    }
    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    std::cout << "������ʱ��: " << duration << "��" 
              << ", �״�ƽ��֡��: " << (captureThreads[0].frameCount / (duration ? duration : 1)) << " fps"
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

void SynchronizedCollector::setSaveConfig(bool saveRadar, bool saveCamera) {
    saveConfig.saveRadar = saveRadar;
    saveConfig.saveCamera = saveCamera;
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