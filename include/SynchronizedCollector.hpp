#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <limits>
#include <queue>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <shared_mutex>
#include "DataReaderFactory.hpp"

// 数据源基类
class DataSource {
public:
    virtual ~DataSource() = default;
    virtual bool init() = 0;
    virtual bool capture(int64_t timestamp) = 0;
    virtual void stop() = 0;
    virtual std::string getSourceName() const = 0;
    virtual int64_t getLastTimestamp() const = 0;
};

// 视频数据源
class VideoSource : public DataSource {
private:
    std::shared_ptr<VideoStreamReader> reader;
    std::string sourceName;
    int64_t lastTimestamp{0};
    cv::Mat lastFrame;  // 添加最后一帧的缓存
    mutable std::shared_mutex dataMutex;        // 针对lastFrame的读写锁
    
public:
    VideoSource(const std::string& videoPath, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void stop() override;
    std::string getSourceName() const override;
    std::shared_ptr<VideoStreamReader> getReader() const { return reader; }
    int64_t getLastTimestamp() const override { return lastTimestamp; }
    const cv::Mat& getLastFrame() const;
};

// 雷达数据源
class RadarSource : public DataSource {
private:
    std::unique_ptr<TcpCommandHandler> handler;
    std::string sourceName;
    int64_t lastTimestamp{0};
    std::vector<TargetInfoParse_0xA8::TargetInfo> lastTargets;
    std::vector<TargetInfoParse_0xA8::TargetTrace> lastTraces;
    int m_modeType = -1;  // 读取数据时获取当前雷达数据输出状态
    mutable std::shared_mutex dataMutex;        // 针对lastTagets的读写锁
    
public:
    RadarSource(const std::string& ip, int port, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void stop() override;
    std::string getSourceName() const override;
    int64_t getLastTimestamp() const override { return lastTimestamp; }

    void saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, const std::string& csv_path, RadarFileReader::Format saveFormat);  // 保存具体雷达数据
    std::vector<TargetInfoParse_0xA8::TargetInfo> getLastTargets() const;
    std::vector<TargetInfoParse_0xA8::TargetTrace> getLastTraces() const;

    void setModeType(int modeType);
    int getModeType() const;
};

// 同步数据采集器
class SynchronizedCollector {
public:
    // 添加数据源时指定类型
    void addSource(std::unique_ptr<DataSource> source, bool isMainSource = false);
    void start();
    void stop();
    void printStats();

private:
    // 保存参数
    struct SavaConfig{
        bool saveRadar = true;
        bool saveCamera = true;
        RadarFileReader::Format saveFormat = RadarFileReader::Format::BIN;
        std::string baseDir;
    };

    // 保存任务
    struct SaveTask{
        int64_t timestamp;
        std::vector<TargetInfoParse_0xA8::TargetInfo> radarData;
        std::vector<std::pair<size_t, cv::Mat>> cameraFrames;
    };

    // 图像帧缓存结构，通过时间戳查找关联帧
    struct ImageFrame {
        cv::Mat frame;
        int64_t timestamp;
        ImageFrame(const cv::Mat& f, int64_t ts) : frame(f.clone()), timestamp(ts) {}
    };

    struct RadarFrame {
        std::vector<TargetInfoParse_0xA8::TargetInfo> targets;
        int64_t timestamp;
        RadarFrame(const std::vector<TargetInfoParse_0xA8::TargetInfo>& f, int64_t ts) : targets(f), timestamp(ts) {};
    };

    struct TraceFrame {
        std::vector<TargetInfoParse_0xA8::TargetTrace> traces;
        int64_t timestamp;
        TraceFrame(const std::vector<TargetInfoParse_0xA8::TargetTrace>& f, int64_t ts) : traces(f), timestamp(ts) {};
    };

    // 通用线程管理
    struct CaptureThread {
        std::thread thread;
        std::atomic<int64_t> lastCaptureTime{0};
        std::atomic<int> frameCount{0};
        std::deque<std::unique_ptr<ImageFrame>> imageFrameBuffer;  // 添加帧缓冲
        std::deque<std::unique_ptr<RadarFrame>> radarFrameBuffer;  // 添加雷达缓冲
        std::deque<std::unique_ptr<TraceFrame>> traceFrameBuffer;  // 添加点迹缓冲
        mutable std::mutex bufferMutex;     // 主要防止相机线程的写入和雷达线程的读取和修改冲突
        const size_t MAX_BUFFER_SIZE = 120;  // 最大缓冲帧数

        // 删除复制构造函数和赋值运算符
        CaptureThread(const CaptureThread&) = delete;
        CaptureThread& operator=(const CaptureThread&) = delete;

        // 添加移动构造函数和移动赋值运算符
        CaptureThread(CaptureThread&& other) noexcept
            : thread(std::move(other.thread))
            , lastCaptureTime(other.lastCaptureTime.load())
            , frameCount(other.frameCount.load()) {}

        CaptureThread& operator=(CaptureThread&& other) noexcept {
            if (this != &other) {
                thread = std::move(other.thread);
                lastCaptureTime = other.lastCaptureTime.load();
                frameCount = other.frameCount.load();
            }
            return *this;
        }

        // 添加默认构造函数
        CaptureThread() = default;
    };

    std::unique_ptr<DataSource> mainSource;  // 主数据源（雷达）
    std::vector<std::unique_ptr<DataSource>> subSources;  // 从数据源（相机）
    std::vector<CaptureThread> captureThreads;  // 对应的采集线程管理
    std::atomic<bool> isRunning{false};
    std::chrono::steady_clock::time_point startTime;
    std::chrono::steady_clock::time_point lastFrameTime;
    // 保存参数
    SavaConfig saveConfig;
    std::queue<SaveTask> saveQueue;
    std::mutex saveMutex;
    std::thread saveThread;     // 保存线程
    // 同步线程
    std::mutex syncMutex;
    std::thread syncThread;

    void mainSourceLoop();  // 主源（雷达）采集循环
    void subSourceLoop(size_t sourceIndex);  // 从源（相机）采集循环
    void saveThreadLoop();  // 保存线程循环
    void syncThreadLoop();  // 同步线程循环

    // int64_t getCurrentTimestamp();
    static std::string getCurrentTimeString();
    cv::Mat findClosestFrame(CaptureThread& thread, int64_t timestamp, bool& radaEraseFlag);  // 查找最近的帧

public:
    void setSaveConfig(bool saveRadar, bool saveCamera, RadarFileReader::Format saveFormat);
    // 以下方法均获取到最新帧的数据，但是有可能不连续。理论上来说，应该在buffer中操作，通过观察buffer来判断是否溢出丢帧。
    int64_t getCurrentTimestamp();
    std::shared_ptr<RadarData> getMainSourceData() const;
    std::shared_ptr<RadarTraceData> getMainSourceTraceData() const;
    std::vector<std::pair<size_t, cv::Mat>> getSubSourceData() const;
    // buffer操作，这里会将buffer中的数据出队，因此需谨慎处理其与保存队列的逻辑。
    std::shared_ptr<RadarTraceData> getMainSourceTraceDataFromBuffer();
};