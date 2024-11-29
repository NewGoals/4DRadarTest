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
    mutable std::shared_mutex dataMutex;        // 针对lastTagets的读写锁
    
public:
    RadarSource(const std::string& ip, int port, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void stop() override;
    std::string getSourceName() const override;
    int64_t getLastTimestamp() const override { return lastTimestamp; }

    void saveData(const std::string& path);     // 保存最近一帧雷达数据
    void saveTargetData(const std::vector<TargetInfoParse_0xA8::TargetInfo>& targets, const std::string& csv_path);  // 保存具体雷达数据
    std::vector<TargetInfoParse_0xA8::TargetInfo> getLastTargets() const;
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

    // 通用线程管理
    struct CaptureThread {
        std::thread thread;
        std::atomic<int64_t> lastCaptureTime{0};
        std::atomic<int> frameCount{0};
        std::deque<std::unique_ptr<ImageFrame>> frameBuffer;  // 添加帧缓冲
        mutable std::mutex bufferMutex;
        const size_t MAX_BUFFER_SIZE = 60;  // 最大缓冲帧数

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
    static constexpr std::chrono::milliseconds FRAME_INTERVAL{33}; // 约30fps
    // 保存参数
    SavaConfig saveConfig;
    std::queue<SaveTask> saveQueue;
    std::mutex saveMutex;
    std::thread saveThread;     // 保存线程

    void mainSourceLoop();  // 主源（雷达）采集循环
    void subSourceLoop(size_t sourceIndex);  // 从源（相机）采集循环
    void saveThreadLoop();  // 保存线程循环

    int64_t getCurrentTimestamp();
    static std::string getCurrentTimeString();
    cv::Mat findClosestFrame(CaptureThread& thread, int64_t timestamp);  // 查找最近的帧

public:
    void setSaveConfig(bool saveRadar, bool saveCamera);
    std::shared_ptr<RadarData> getMainSourceData() const;
    std::vector<std::pair<size_t, cv::Mat>> getSubSourceData() const;
};