#pragma once
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
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
};

// 视频数据源
class VideoSource : public DataSource {
private:
    std::shared_ptr<VideoStreamReader> reader;
    std::string savePath;
    std::string sourceName;
    
public:
    VideoSource(const std::string& videoPath, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void setSavePath(const std::string& path) override;
    void stop() override;
    std::string getSourceName() const override;
};

// 雷达数据源
class RadarSource : public DataSource {
private:
    std::unique_ptr<TcpCommandHandler> handler;
    std::string savePath;
    std::string sourceName;
    
public:
    RadarSource(const std::string& ip, int port, const std::string& name);
    bool init() override;
    bool capture(int64_t timestamp) override;
    void setSavePath(const std::string& path) override;
    void stop() override;
    std::string getSourceName() const override;
};

// 同步数据采集器
class SynchronizedCollector {
public:
    void addSource(std::unique_ptr<DataSource> source);
    void start();
    void stop();
    void printStats();

private:
    struct CaptureThread {
        std::thread thread;
        std::atomic<int64_t> lastCaptureTime{0};
        std::atomic<int> frameCount{0};

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

    std::vector<std::unique_ptr<DataSource>> sources;              // 存储所有数据源
    std::vector<CaptureThread> captureThreads;                     // 对应的采集线程   
    std::atomic<bool> isRunning{false};                            // 采集器运行状态
    std::chrono::steady_clock::time_point startTime;               // 开始时间
    std::string baseDir;                                           // 数据保存基础目录

    void captureLoop(size_t sourceIndex);
    int64_t getCurrentTimestamp();
    static std::string getCurrentTimeString();
};