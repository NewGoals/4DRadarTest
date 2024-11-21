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

// ����Դ����
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

// ��Ƶ����Դ
class VideoSource : public DataSource {
private:
    std::shared_ptr<VideoStreamReader> reader;
    std::string savePath;
    std::string sourceName;
    int64_t lastTimestamp{0};
    cv::Mat lastFrame;  // ������һ֡�Ļ���
    
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

// �״�����Դ
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

// ͬ�����ݲɼ���
class SynchronizedCollector {
public:
    // �������Դʱָ������
    void addSource(std::unique_ptr<DataSource> source, bool isMainSource = false);
    void start();
    void stop();
    void printStats();

private:
    // ͼ��֡����ṹ
    struct ImageFrame {
        cv::Mat frame;
        int64_t timestamp;
        ImageFrame(const cv::Mat& f, int64_t ts) : frame(f.clone()), timestamp(ts) {}
    };

    struct CaptureThread {
        std::thread thread;
        std::atomic<int64_t> lastCaptureTime{0};
        std::atomic<int> frameCount{0};
        std::queue<std::unique_ptr<ImageFrame>> frameBuffer;  // ���֡����
        mutable std::mutex bufferMutex;
        const size_t MAX_BUFFER_SIZE = 30;  // ��󻺳�֡��

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

    std::unique_ptr<DataSource> mainSource;  // ������Դ���״
    std::vector<std::unique_ptr<DataSource>> subSources;  // ������Դ�������
    std::vector<CaptureThread> captureThreads;  // ��Ӧ�Ĳɼ��߳�
    std::atomic<bool> isRunning{false};
    std::chrono::steady_clock::time_point startTime;
    std::string baseDir;
    std::chrono::steady_clock::time_point lastFrameTime;
    static constexpr std::chrono::milliseconds FRAME_INTERVAL{33}; // Լ30fps

    void mainSourceLoop();  // ��Դ���״�ɼ�ѭ��
    void subSourceLoop(size_t sourceIndex);  // ��Դ��������ɼ�ѭ��
    int64_t getCurrentTimestamp();
    static std::string getCurrentTimeString();
    void saveFrames(int64_t timestamp);  // ����ͬ��֡
    cv::Mat findClosestFrame(CaptureThread& thread, int64_t timestamp);  // ���������֡
};