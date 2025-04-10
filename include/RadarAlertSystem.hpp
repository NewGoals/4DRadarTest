// RadarProcessor.h
#pragma once
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include <functional>
#include <deque>
#include <vector>
#include <atomic>
#include <mutex>
#include "SensorData.hpp"

//==============================================================================
// 点云预警策略
//==============================================================================
struct RadarAlertConfig
{
    // 坐标系参数
    Eigen::Matrix4f radar_to_world = Eigen::Matrix4f::Identity();

    // 处理参数
    float max_detection_range = 200.0f;  // 最大检测距离（米）
    float min_detection_range = 10.0f;
    float velocity_filter_thresh = 0.3f; // 速度波动阈值（m/s）
    float update_interval = 0.1f;        // 处理间隔（秒)，衰减策略
};

class RadarAlertSystem
{
public:
    // 防区形状类型
    enum class ZoneType
    {
        CUBOID,
        CYLINDER,
        POLYGON,
        ROTATED_CUBOID,
        ROTATED_CUBOID_3DOF
    };

    // 防区配置结构
    struct DefenseZone
    {
        int zone_id;
        ZoneType shape;
        std::vector<Eigen::Vector3f> params; // 形状参数
        float min_speed = 0.0f;
        float max_speed = 100.0f;
        int min_points = 3;            // 最小点数
        float trigger_duration = 1.0f; // 触发时长阈值，目标存在1秒即触发报警
    };

    // 报警回调类型
    using AlertCallback = std::function<void(int zone_id, bool status)>;

    RadarAlertSystem(const RadarAlertConfig &config);
    ~RadarAlertSystem();

    // 防区管理
    void addDefenseZone(const DefenseZone &zone);
    void updateDefenseZone(int zone_id, const DefenseZone &new_config);
    void removeDefenseZone(int zone_id);
    void clearAllZones();

    // 数据处理
    void processFrame(const std::vector<Eigen::Vector3f> &raw_points,
                      const std::vector<float> &velocities,
                      double timestamp);

    // 状态获取
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr getVisualizationCloud() const;
    void getAlertStatus(std::map<int, bool> &status_map) const;

    // 设置回调
    void setAlertCallback(const AlertCallback &callback);

    // 新增：获取当前所有防区配置（返回防区配置的副本）
    std::vector<DefenseZone> getDefenseZones();

private:
    // 数据容器
    struct ProcessedPoint
    {
        Eigen::Vector3f position;
        float velocity;
        double timestamp;
    };

    // 防区状态
    struct ZoneStatus
    {
        DefenseZone config;
        bool is_alert = false;
        float duration = 0.0f; // 目标存在时间
        float dynamic_decay = 0.1;
        std::vector<Eigen::Vector3f> trigger_points;
    };

    // 点云状态, 是否选择合并雷达帧
    struct PointStatus
    {
        bool is_combine = false;
        int frame_count = 3;
        std::deque<std::vector<RadarPoint>> combinRadarData_;
    };

    // 核心成员
    RadarAlertConfig config_;
    std::atomic<bool> running_{true};
    mutable std::mutex data_mutex_;

    // 空间索引
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // 防区管理
    std::vector<ZoneStatus> defense_zones_;
    AlertCallback alert_callback_;

    // 点云合并管理
    PointStatus combine_points_;

    // 处理逻辑
    void transformCoordinates(std::vector<ProcessedPoint> &output,
                              const std::vector<Eigen::Vector3f> &input,
                              const std::vector<float> &velocities,
                              double timestamp);
    void updateSpatialIndex(const std::vector<ProcessedPoint> &points);
    void checkZones(const std::vector<ProcessedPoint> &points);
    bool isInZone(const ZoneStatus &zone, const Eigen::Vector3f &point) const;
};


//==============================================================================
// 轨迹预警策略
//==============================================================================
struct RadarTraceAlertConfig
{
    // 坐标系参数
    Eigen::Matrix4f radar_to_world = Eigen::Matrix4f::Identity();
    // 处理参数
    float max_detection_range = 250.0f;  // 最大检测距离（米）
    float min_detection_range = 0.0f;   // 最小检测距离（米）
    float velocity_filter_thresh = 0.3f; // 速度波动阈值（m/s）
    float update_interval = 0.1f;        // 处理间隔（秒)，衰减策略
    int min_continuous_frames = 10;      // 最小连续帧数阈值
};

class RadarTraceAlertSystem{
public:
    // 防区形状类型
    enum class ZoneType
    {
        CUBOID,
        CYLINDER,
        POLYGON,
        ROTATED_CUBOID,
        ROTATED_CUBOID_3DOF
    };

    // 防区配置结构
    struct DefenseZone
    {
        int zone_id;
        ZoneType shape;
        std::vector<Eigen::Vector3f> params; // 形状参数
        float min_speed = 0.0f;
        float max_speed = 100.0f;
        int min_points = 3;            // 最小点数
        float trigger_duration = 1.0f; // 触发时长阈值，目标存在1秒即触发报警
    };

    // 报警回调类型
    using AlertCallback = std::function<void(int zone_id, bool status, uint32_t trace_id)>;

    RadarTraceAlertSystem(const RadarTraceAlertConfig &config);
    ~RadarTraceAlertSystem();

    // 防区管理
    void addDefenseZone(const DefenseZone &zone);
    void updateDefenseZone(int zone_id, const DefenseZone &new_config);
    void removeDefenseZone(int zone_id);
    void clearAllZones();

    // 数据处理
    void processFrame(const RadarTraceData &trace_data);

    // 状态获取
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr getVisualizationCloud() const;
    void getAlertStatus(std::map<int, std::vector<uint32_t>> &status_map) const;

    // 设置回调
    void setAlertCallback(const AlertCallback &callback);

    // 获取当前所有防区配置（返回防区配置的副本）
    std::vector<DefenseZone> getDefenseZones() const;

private:
    // 处理后的轨迹点
    struct ProcessedTrace
    {
        uint32_t ID;
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        double timestamp;
        float SNR;
        float length;
    };

    // 轨迹历史记录
    struct TraceHistory
    {
        uint32_t trace_id;
        std::deque<ProcessedTrace> positions;
        int continuous_frames; // 连续出现的帧数
        double last_update_time;
    };

    // 防区状态
    struct ZoneStatus
    {
        DefenseZone config;
        bool is_alert = false;
        std::map<uint32_t, bool> alert_traces; // 跟踪每个轨迹ID的告警状态
        std::vector<uint32_t> current_trace_ids; // 当前在区域内的轨迹ID
    };

    // 核心成员
    RadarTraceAlertConfig config_;
    std::atomic<bool> running_{true};
    mutable std::mutex data_mutex_;

    // 空间索引
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // 防区管理
    std::vector<ZoneStatus> defense_zones_;

    // 轨迹历史管理
    std::map<uint32_t, TraceHistory> trace_history_;
    double last_cleanup_time_; // 上次清理过期轨迹的时间

    // 回调函数
    AlertCallback alert_callback_;

    // 处理逻辑
    std::vector<ProcessedTrace> transformTraces(const RadarTraceData &trace_data);
    void updateTraceHistory(const std::vector<ProcessedTrace> &traces, double current_time);
    void updateSpatialIndex(const std::vector<ProcessedTrace> &traces);
    void checkZones(double current_time);
    bool isInZone(const ZoneStatus &zone, const Eigen::Vector3f &point) const;
    void cleanupExpiredTraces(double current_time);
};