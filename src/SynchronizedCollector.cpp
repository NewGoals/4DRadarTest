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

    // std::cout << "getcurrent time: " << std::dynamic_pointer_cast<ImageData>(data)->timestamp << std::endl;
    // std::cout << "cap time: " << timestamp << std::endl;
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
    if (!handler->receiveAndParseFrame(result)){
        // std::cout << "RadarSource::capture error: hanlder->receiveAndParse. " << std::endl;
        return false;
    }
    
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

    // ���������̣߳� ͬ���߳�
    if(saveConfig.saveRadar || saveConfig.saveCamera){
        // ������������Ŀ¼
        saveConfig.baseDir = "sync_data_" + getCurrentTimeString();
        std::filesystem::create_directory(saveConfig.baseDir);
        
        std::cout << "�������ݱ���Ŀ¼: " << saveConfig.baseDir << std::endl;
        saveThread = std::thread(&SynchronizedCollector::saveThreadLoop, this);

        // ����ͬ���߳�
        syncThread = std::thread(&SynchronizedCollector::syncThreadLoop, this);
    }
    else{
        std::cout << "δ���ñ����߳�" << std::endl;
    }
    
    std::cout << "���вɼ��߳�������" << std::endl;
}

void SynchronizedCollector::mainSourceLoop() {
    auto& thread = captureThreads[0];   // ��Դ�߳�
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        
        if (auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())) {
            if (radarSource->capture(timestamp)) {
                thread.frameCount++;
                thread.lastCaptureTime = timestamp;

                {
                    std::lock_guard<std::mutex> lock(thread.bufferMutex);
                    // ���������������ɾ�������֡
                    if (thread.radarFrameBuffer.size() >= thread.MAX_BUFFER_SIZE) {
                        thread.radarFrameBuffer.pop_front();
                    }
                    // ������֡��ӵ�������
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
                    // ���������������ɾ�������֡
                    if (thread.imageFrameBuffer.size() >= thread.MAX_BUFFER_SIZE) {
                        thread.imageFrameBuffer.pop_front();
                    }
                    // ������֡��ӵ�������
                    int delay_time = 600;   // �ӳ�����Ϊ600ms
                    auto newFrame = std::make_unique<ImageFrame>(videoSource->getLastFrame(), timestamp - 600);
                    thread.imageFrameBuffer.push_back(std::move(newFrame));
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
            if(saveConfig.saveFormat == RadarFileReader::Format::CSV){
                std::string dataPath = saveConfig.baseDir + "/" + mainSource->getSourceName() + "/" + mainSource->getSourceName() + "_" + std::to_string(task.timestamp) + ".csv";
                // ������Ҫ����handler��saveTargetData����������task.radarData��������ʹ��mainSource->saveData�����������һ֡����
                if(auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())){
                    radarSource->saveTargetData(task.radarData, dataPath, RadarFileReader::Format::CSV);
                }
            }
            else if(saveConfig.saveFormat == RadarFileReader::Format::BIN){
                std::string dataPath = saveConfig.baseDir + "/" + mainSource->getSourceName() + "/" + mainSource->getSourceName() + "_" + std::to_string(task.timestamp) + ".bin";
                // ������Ҫ����handler��saveTargetData����������task.radarData��������ʹ��mainSource->saveData�����������һ֡����
                if(auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())){
                    radarSource->saveTargetData(task.radarData, dataPath, RadarFileReader::Format::BIN);
                }
            }
            
        }

        // �����������
        if(saveConfig.saveCamera){
            for(const auto& [sourceIndex, frame] : task.cameraFrames){
                if(!frame.empty()){
                    std::string framePath = saveConfig.baseDir + "/" + 
                                    subSources[sourceIndex]->getSourceName() + "/" +
                                    subSources[sourceIndex]->getSourceName() + "_" + 
                                    std::to_string(task.timestamp) + ".bmp";

                    // �����ʱ
                    // auto start = std::chrono::high_resolution_clock::now();
                    
                    cv::imwrite(framePath, frame);

                    // auto end = std::chrono::high_resolution_clock::now(); // ��¼����ʱ��
                    // std::chrono::duration<double, std::milli> elapsed = end - start; // ����ʱ����Ϊ���뼶
                    
                    // std::cout << "ͼ��д���ʱ: " << elapsed.count() << " ms" << std::endl; // ���д���ʱ
                }
            }
        }
    }
}

void SynchronizedCollector::syncThreadLoop(){
    while (isRunning) {
        // ���״�������ӵ�����������
        SaveTask task;
        
        // ȡ�״�����ʱ��
        int64_t radarTimestamp;
        {
            auto& mainThread = captureThreads[0];
            std::lock_guard<std::mutex> lock(mainThread.bufferMutex);
            if (mainThread.radarFrameBuffer.empty()) {
                // std::cout << "�״���л���Ϊ��" << std::endl;
                continue;
            }
            radarTimestamp = mainThread.radarFrameBuffer.front()->timestamp;

            // ���task
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
                    // ͼ��Ϊ�գ���ɾ���״�����
                    // ȥ������
                    {
                        auto& mainThread = captureThreads[0];
                        std::lock_guard<std::mutex> lock(mainThread.bufferMutex);
                        mainThread.radarFrameBuffer.erase(mainThread.radarFrameBuffer.begin());       
                    }
                    // �������ĳһ֡ƥ�䲻�ˣ����״����ݹ�ʱ��ֱ���˳�
                    flag = true;
                    break;
                }
                else{
                    // �ȴ�ͼ�����
                    flag = true;
                    break;
                }
            }
            if(flag){
                // �����б�����������ǵȴ�ͼ�����
                continue;
            }
            else{
                // ����ͬ��������ͼ������ӽ�task����Ҫ���״�����ɾ��
                {
                    auto& mainThread = captureThreads[0];
                    std::lock_guard<std::mutex> lock(mainThread.bufferMutex);
                    mainThread.radarFrameBuffer.erase(mainThread.radarFrameBuffer.begin());       
                }
            }
        }

        {
            // ������������ӵ�������
            std::lock_guard<std::mutex> lock(saveMutex);
            saveQueue.push(std::move(task)); 
        }
    }
}


cv::Mat SynchronizedCollector::findClosestFrame(CaptureThread& thread, int64_t timestamp, bool& radarEraseFlag) {
    std::lock_guard<std::mutex> lock(thread.bufferMutex);
    
    if (thread.imageFrameBuffer.empty()) {
        // std::cout << "��ǰ֡����Ϊ��" << std::endl;
        return cv::Mat();
    }
    
    static const int64_t MAX_TIME_DIFF = 50;  // 50ms���̶�
    static const size_t MAX_BUFFER_RETAIN = 10;  // ���������֡��
    size_t closestIndex = -1;
    int64_t minDiff = (std::numeric_limits<int64_t>::max)();    // ��ʼ����С��ֵΪ�������ֵ
    int64_t prevDiff = (std::numeric_limits<int64_t>::max)();
    bool returnFlag = false;
    
    // �ҵ���ӽ���֡��������֮ǰ����֮���
    // std::cout << "current image buffer size: " << thread.imageFrameBuffer.size() << std::endl;
    for (size_t i = 0; i < thread.imageFrameBuffer.size(); ++i) {
        auto& currentFrame = thread.imageFrameBuffer[i];
        int64_t diff = std::abs(currentFrame->timestamp - timestamp);

        // �����ֵ��ʼ�����ֹͣ����
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

    // ���û���ҵ����ʵ�֡��֤����ʱ�˳�ԭ��Ϊ��ֵ�������˳����Ҷ����е�ͼ��Ҫ���״����ݸ��£����ﲻ���κβ�����ֱ�ӷ��صȴ��״����ݣ�
    if (minDiff > MAX_TIME_DIFF && returnFlag) {
        // ��ʱ�״������Ѿ���ʱ��Ӧ�ö����״����ݣ�����ͼ������
        radarEraseFlag = true;
        return cv::Mat();
    }
    else if (minDiff < MAX_TIME_DIFF && returnFlag)
    {
        // δ�������˳��������ҵ���Ӧͼ��֡��ɾ��֮ǰ��ͼ��
        thread.imageFrameBuffer.erase(
            thread.imageFrameBuffer.begin(), 
            thread.imageFrameBuffer.begin() + closestIndex
        );
        if (!thread.imageFrameBuffer.empty()) {
            auto closestFramePtr = std::move(thread.imageFrameBuffer.front());
            if (std::abs(closestFramePtr->timestamp - timestamp) <= MAX_TIME_DIFF) {
                // ֱ���ƶ����أ������¡
                cv::Mat result = std::move(closestFramePtr->frame);
                thread.imageFrameBuffer.erase(thread.imageFrameBuffer.begin());
                return result;
            }
        }
    }
    else{
        // ��һ����ʾ������ɣ�������û���ҵ���Ӧ��ͼ�񣬶�Ӧ�õȴ�ͼ�����ݸ���
        return cv::Mat();
    }
    
    // һ�㲻�ᵽ������ǻ��ǵ������
    std::cout << "��ȡ����֡�������⣬Ϊ��" << std::endl;
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
                  << "��֡��: " << mainThread->frameCount
                  << ", ����֡��: " << mainThread->radarFrameBuffer.size()
                  << ", ���ɼ�ʱ��: " << mainThread->lastCaptureTime << "ms"
                  << ", ����������: " << (mainThread->radarFrameBuffer.size() * 100.0 / mainThread->MAX_BUFFER_SIZE) << "%"
                  << std::endl;
        {
            std::lock_guard<std::mutex> lock(mainThread->bufferMutex);
            if(!mainThread->radarFrameBuffer.empty()){
               std::cout << "�״���ף�" << mainThread->radarFrameBuffer.front()->timestamp
                      << ", �״��β��" << mainThread->radarFrameBuffer.back()->timestamp
                      << std::endl; 
            }
        }
    }
    
    // ��ӡ������Դ״̬
    for (size_t i = 0; i < subSources.size(); ++i) {
        auto& thread = captureThreads[i + 1]; // +1 ��Ϊ��һ����������Դ
        std::cout << "������Դ (" << subSources[i]->getSourceName() << "): "
                  << "��֡��: " << thread.frameCount
                  << ", ����֡��: " << thread.imageFrameBuffer.size()
                  << ", ���ɼ�ʱ��: " << thread.lastCaptureTime << "ms"
                  << ", ����������: " << (thread.imageFrameBuffer.size() * 100.0 / thread.MAX_BUFFER_SIZE) << "%"
                  << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(thread.bufferMutex);
            if(!thread.imageFrameBuffer.empty()){
               std::cout << "ͼ����ף�" << thread.imageFrameBuffer.front()->timestamp
                      << ", ͼ���β��" << thread.imageFrameBuffer.back()->timestamp
                      << std::endl; 
            }
        }
    }

    
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - startTime).count();
    std::cout << "������ʱ��: " << static_cast<int>(duration) << "��" 
              << ", �״�ƽ��֡��: " << static_cast<int>((captureThreads[0].frameCount / (duration ? duration : 1))) << " fps"
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