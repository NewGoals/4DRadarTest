#include "SynchronizedCollector.hpp"
#include <filesystem>
#include <sstream>
#include <iomanip>

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
    if (!reader->grabFrame()) return false;
    
    auto data = reader->getData();
    if (!data) return false;
    
    cv::Mat& frame = std::dynamic_pointer_cast<ImageData>(data)->frame;
    std::string framePath = savePath + "/" + sourceName + "_" + 
                           std::to_string(timestamp) + ".jpg";
    cv::imwrite(framePath, frame);
    return true;
}

void VideoSource::setSavePath(const std::string& path) {
    savePath = path;
    std::filesystem::create_directory(savePath);
}

void VideoSource::stop() {
    // 清理资源
}

std::string VideoSource::getSourceName() const {
    return sourceName;
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
    CommandParseResult result;
    if (!handler->receiveAndParseFrame(result)) return false;
    
    if (auto* targets = std::get_if<std::vector<TargetInfoParse_0xA8::TargetInfo>>(&result)) {
        std::string dataPath = savePath + "/" + sourceName + "_" + 
                             std::to_string(timestamp) + ".csv";
        handler->startRecording(dataPath);
        handler->saveTargetData(*targets);
        handler->stopRecording();
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


//==============================================================================
// SynchronizedCollector 实现
//==============================================================================
void SynchronizedCollector::addSource(std::unique_ptr<DataSource> source) {
    sources.push_back(std::move(source));
}

void SynchronizedCollector::start() {
    if (sources.empty()) throw std::runtime_error("没有数据源");
    
    baseDir = "sync_data_" + getCurrentTimeString();
    std::filesystem::create_directory(baseDir);
    
    for (auto& source : sources) {
        if (!source->init()) {
            throw std::runtime_error("初始化数据源失败: " + source->getSourceName());
        }
        std::string sourceDir = baseDir + "/" + source->getSourceName();
        source->setSavePath(sourceDir);
    }
    
    isRunning = true;
    startTime = std::chrono::steady_clock::now();
    
    for (size_t i = 0; i < sources.size(); ++i) {
        captureThreads.emplace_back();
        auto& ct = captureThreads.back();
        ct.thread = std::thread(&SynchronizedCollector::captureLoop, this, i);
    }
}

void SynchronizedCollector::stop() {
    isRunning = false;
    for (auto& ct : captureThreads) {
        if (ct.thread.joinable()) {
            ct.thread.join();
        }
    }
    for (auto& source : sources) {
        source->stop();
    }
}

void SynchronizedCollector::printStats() {
    auto currentTime = std::chrono::steady_clock::now();
    double elapsedSeconds = std::chrono::duration<double>(currentTime - startTime).count();
    
    for (size_t i = 0; i < sources.size(); ++i) {
        double fps = captureThreads[i].frameCount / elapsedSeconds;
        std::cout << sources[i]->getSourceName() 
                  << " 已采集帧数: " << captureThreads[i].frameCount 
                  << " 帧率: " << std::fixed << std::setprecision(2) << fps << " fps"
                  << std::endl;
    }
}

void SynchronizedCollector::captureLoop(size_t sourceIndex) {
    auto& source = sources[sourceIndex];
    auto& stats = captureThreads[sourceIndex];
    
    while (isRunning) {
        int64_t timestamp = getCurrentTimestamp();
        if (source->capture(timestamp)) {
            stats.lastCaptureTime = timestamp;
            stats.frameCount++;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
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