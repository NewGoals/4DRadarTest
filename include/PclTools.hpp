#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <omp.h>
#include <chrono>
#include <iostream>
#include <unordered_set>
#include "SensorData.hpp"
#include "RadarAlertSystem.hpp"

class PCLTools
{
public:
    // 构造函数
    PCLTools() = default;

    // 计时器函数
    template <typename Func, typename... Args>
    static auto timeFunction(Func &&func, Args &&...args);

    // DBSCAN 主函数
    static int dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, double eps, int min_pts);

    // 通用函数工具
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Raw2PointRGB_vr(const std::shared_ptr<RadarData> &radar_data);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RadarTraceData2PointRGB(const std::shared_ptr<RadarTraceData> &radar_trace_data);
    void PointViewerInit(pcl::visualization::PCLVisualizer::Ptr viewer);                                                                                                    // 初始化网格线
    void RenderAccumulatedData(pcl::visualization::PCLVisualizer::Ptr viewer, const std::deque<std::shared_ptr<RadarTraceData>> &data_sequence, size_t latest_frames = 10); // 渲染轨迹

    // 轨迹告警通用函数
    void addDefenseZoneVisualization(pcl::visualization::PCLVisualizer::Ptr viewer,
                                     const RadarTraceAlertSystem::DefenseZone &zone);
    void updateAlertVisualization(pcl::visualization::PCLVisualizer::Ptr viewer,
                                  const RadarTraceAlertSystem &alert_system,
                                  const std::map<int, std::vector<uint32_t>> &alert_status);

private:
    // DBSCAN 嵌套类
    class DBSCAN
    {
    public:
        DBSCAN(double eps, int min_pts) : eps_(eps), min_pts_(min_pts) {};
        int run(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud);

    private:
        double eps_;
        int min_pts_;

        std::vector<int> regionQuery(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, pcl::KdTreeFLANN<pcl::PointXYZL> &kdtree, const pcl::PointXYZL &point);
        void labelPoint(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, const std::vector<int> &points, int label);
        int expandCluster(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, pcl::KdTreeFLANN<pcl::PointXYZL> &kdtree, int point_idx, int cluster_id);
        void removeNoisePoints(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud);
    };
};
