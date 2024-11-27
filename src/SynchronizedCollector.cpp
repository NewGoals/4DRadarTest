#include "SynchronizedCollector.hpp"
#include <filesystem>
#include <sstream>
#include <iomanip>

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
    if (!reader->grabFrame()) {
        return false;
    }

    auto data = reader->getData();
    if (!data) return false;
    
    lastFrame = std::dynamic_pointer_cast<ImageData>(data)->frame.clone();
    lastTimestamp = timestamp;  // ��¼ʵ�ʵĲɼ�ʱ������������ܺ����õ�����ʱ����������
    return true;
}

void VideoSource::setSavePath(const std::string& path) {
    savePath = path;
    std::filesystem::create_directory(savePath);
}

void VideoSource::stop() {
    // ������Դ
}

std::string VideoSource::getSourceName() const {
    return sourceName;
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
    CommandParseResult result;
    if (!handler->receiveAndParseFrame(result)) return false;
    
    if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
        lastTargets = *targets;  // �������ݵ���Ա����
        lastTimestamp = timestamp;
        return true;
    }
    return false;
}

void RadarSource::setSavePath(const std::string& path) {
    savePath = path;
    std::filesystem::create_directory(savePath);
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
    
    // ������������Ŀ¼
    baseDir = "sync_data_" + getCurrentTimeString();
    std::filesystem::create_directory(baseDir);
    
    std::cout << "�������ݱ���Ŀ¼: " << baseDir << std::endl;
    
    // ����������Դ�߳�
    captureThreads.emplace_back();
    captureThreads.back().thread = std::thread(&SynchronizedCollector::mainSourceLoop, this);
    
    // ����������Դ�߳�
    for (size_t i = 0; i < subSources.size(); ++i) {
        captureThreads.emplace_back();
        captureThreads.back().thread = std::thread(&SynchronizedCollector::subSourceLoop, this, i);
    }
    
    std::cout << "���вɼ��߳�������" << std::endl;
}

void SynchronizedCollector::mainSourceLoop() {
    auto& thread = captureThreads[0];
    
    std::string sourcePath = baseDir + "/" + mainSource->getSourceName();
    std::filesystem::create_directory(sourcePath);
    mainSource->setSavePath(sourcePath);
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        
        if (auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())) {
            if (radarSource->capture(timestamp)) {
                // ʹ��ʵ�ʵĲɼ�ʱ���
                std::string dataPath = sourcePath + "/" + mainSource->getSourceName() + "_" + 
                                     std::to_string(timestamp) + ".csv";
                
                thread.frameCount++;
                thread.lastCaptureTime = timestamp;
                saveFrames(timestamp);  // ʹ����ͬʱ���������������
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SynchronizedCollector::subSourceLoop(size_t sourceIndex) {
    auto& source = subSources[sourceIndex];
    auto& thread = captureThreads[sourceIndex + 1];
    
    std::string sourcePath = baseDir + "/" + source->getSourceName();
    std::filesystem::create_directory(sourcePath);
    source->setSavePath(sourcePath);
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        
        if (auto* videoSource = dynamic_cast<VideoSource*>(source.get())) {
            if (videoSource->capture(timestamp)) {
                thread.frameCount++;
                thread.lastCaptureTime = timestamp;
                
                std::lock_guard<std::mutex> lock(thread.bufferMutex);
                if (thread.frameBuffer.size() >= thread.MAX_BUFFER_SIZE) {
                    thread.frameBuffer.pop();
                }
                thread.frameBuffer.push(
                    std::make_unique<ImageFrame>(videoSource->getLastFrame(), timestamp)
                );
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SynchronizedCollector::saveFrames(int64_t timestamp) {
    static int totalAttempts = 0;
    static int successfulMatches = 0;
    
    // �����״�����
    if (auto* radarSource = dynamic_cast<RadarSource*>(mainSource.get())) {
        std::string dataPath = baseDir + "/" + mainSource->getSourceName() + "/" +
                              mainSource->getSourceName() + "_" + 
                              std::to_string(timestamp) + ".csv";
        radarSource->saveData(dataPath);
        totalAttempts++;
    }
    
    // �����������
    for (size_t i = 0; i < subSources.size(); ++i) {
        cv::Mat frame = findClosestFrame(captureThreads[i + 1], timestamp);
        if (!frame.empty()) {
            std::string framePath = baseDir + "/" + subSources[i]->getSourceName() + "/" +
                                  subSources[i]->getSourceName() + "_" + 
                                  std::to_string(timestamp) + ".jpg";
            cv::imwrite(framePath, frame);
            successfulMatches++;
        }
    }
    
    if (totalAttempts % 100 == 0) {  // ÿ100֡��ӡһ��ͳ��
        std::cout << "ͬ����: " << (successfulMatches * 100.0 / totalAttempts) 
                  << "% (" << successfulMatches << "/" << totalAttempts << ")" << std::endl;
    }
}

cv::Mat SynchronizedCollector::findClosestFrame(CaptureThread& thread, int64_t timestamp) {
    std::lock_guard<std::mutex> lock(thread.bufferMutex);
    
    if (thread.frameBuffer.empty()) {
        return cv::Mat();
    }
    
    cv::Mat bestFrame;
    int64_t minDiff = (std::numeric_limits<int64_t>::max)();
    static const int64_t MAX_TIME_DIFF = 50;  // 50ms���̶�
    
    // �ҵ���ӽ���֡��������֮ǰ����֮���
    for (size_t i = 0; i < thread.frameBuffer.size(); ++i) {
        int64_t diff = std::abs(thread.frameBuffer.front()->timestamp - timestamp);
        if (diff < minDiff) {
            minDiff = diff;
            if (diff <= MAX_TIME_DIFF) {  // ֻ���������̷�Χ�ڵ�֡
                bestFrame = thread.frameBuffer.front()->frame.clone();
            }
        }
        // ѭ���ƶ�����
        thread.frameBuffer.push(std::move(thread.frameBuffer.front()));
        thread.frameBuffer.pop();
    }
    
    return bestFrame;
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
              << ", ƽ��֡��: " << (captureThreads[0].frameCount / (duration ? duration : 1)) << " fps"
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