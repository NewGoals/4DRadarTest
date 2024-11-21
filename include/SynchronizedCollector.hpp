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

// ����Դ����
class DataSource {
public:
    virtual ~DataSource() = default;
    virtual bool init() = 0;
    virtual bool capture(int64_t timestamp) = 0;
    virtual void stop() = 0;
    virtual std::string getSourceName() const = 0;
    virtual void setSavePath(const std::string& path) = 0;
};

// ��Ƶ����Դ
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

// �״�����Դ
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

// ͬ�����ݲɼ���
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

        // ɾ�����ƹ��캯���͸�ֵ�����
        CaptureThread(const CaptureThread&) = delete;
        CaptureThread& operator=(const CaptureThread&) = delete;

        // ����ƶ����캯�����ƶ���ֵ�����
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

        // ���Ĭ�Ϲ��캯��
        CaptureThread() = default;
    };

    std::vector<std::unique_ptr<DataSource>> sources;              // �洢��������Դ
    std::vector<CaptureThread> captureThreads;                     // ��Ӧ�Ĳɼ��߳�   
    std::atomic<bool> isRunning{false};                            // �ɼ�������״̬
    std::chrono::steady_clock::time_point startTime;               // ��ʼʱ��
    std::string baseDir;                                           // ���ݱ������Ŀ¼

    void captureLoop(size_t sourceIndex);
    int64_t getCurrentTimestamp();
    static std::string getCurrentTimeString();
};