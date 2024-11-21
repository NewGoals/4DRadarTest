#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <limits>
#include <queue>
#include <opencv2/opencv.hpp>
#include "DataReaderFactory.hpp"
#include "TcpCommandHandler.hpp"

// 数据源基类
class DataSource {
public:
    virtual ~DataSource() = default;
    virtual bool init() = 0;
    virtual bool capture(int64_t timestamp) = 0;
    virtual void stop() = 0;
    virtual std::string getSourceName() const = 0;
    virtual void setSavePath(const std::string& path) = 0;
    virtual int64_t getLastTimestamp() const = 0;
};

// 视频数据源
class VideoSource : public DataSource {
private:
    std::shared_ptr<VideoStreamReader> reader;
    std::string savePath;
    std::string sourceName;
    int64_t lastTimestamp{0};
    cv::Mat lastFrame;  // 添加最后一帧的缓存
    
public:
    VideoSource(const std::string& videoPath, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void setSavePath(const std::string& path) override;
    void stop() override;
    std::string getSourceName() const override;
    std::shared_ptr<VideoStreamReader> getReader() const { return reader; }
    int64_t getLastTimestamp() const override { return lastTimestamp; }
    const cv::Mat& getLastFrame() const { return lastFrame; }
};

// 雷达数据源
class RadarSource : public DataSource {
private:
    std::unique_ptr<TcpCommandHandler> handler;
    std::string savePath;
    std::string sourceName;
    int64_t lastTimestamp{0};
    std::vector<TargetInfoParse_0xA8::TargetInfo> lastTargets;
    
public:
    RadarSource(const std::string& ip, int port, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void setSavePath(const std::string& path) override;
    void stop() override;
    std::string getSourceName() const override;
    int64_t getLastTimestamp() const override { return lastTimestamp; }
    void saveData(const std::string& path);
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
    // 图像帧缓存结构
    struct ImageFrame {
        cv::Mat frame;
        int64_t timestamp;
        ImageFrame(const cv::Mat& f, int64_t ts) : frame(f.clone()), timestamp(ts) {}
    };

    struct CaptureThread {
        std::thread thread;
        std::atomic<int64_t> lastCaptureTime{0};
        std::atomic<int> frameCount{0};
        std::queue<std::unique_ptr<ImageFrame>> frameBuffer;  // 添加帧缓冲
        mutable std::mutex bufferMutex;
        const size_t MAX_BUFFER_SIZE = 30;  // 最大缓冲帧数

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
    std::vector<CaptureThread> captureThreads;  // 对应的采集线程
    std::atomic<bool> isRunning{false};
    std::chrono::steady_clock::time_point startTime;
    std::string baseDir;
    std::chrono::steady_clock::time_point lastFrameTime;
    static constexpr std::chrono::milliseconds FRAME_INTERVAL{33}; // 约30fps

    void mainSourceLoop();  // 主源（雷达）采集循环
    void subSourceLoop(size_t sourceIndex);  // 从源（相机）采集循环
    int64_t getCurrentTimestamp();
    static std::string getCurrentTimeString();
    void saveFrames(int64_t timestamp);  // 保存同步帧
    cv::Mat findClosestFrame(CaptureThread& thread, int64_t timestamp);  // 查找最近的帧
};