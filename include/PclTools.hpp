#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <omp.h>
#include <chrono>
#include <iostream>

class PCLTools{
public:
    // 构造函数
    PCLTools() = default;

    // 计时器函数
    template<typename Func, typename... Args>
    static auto timeFunction(Func&& func, Args&&... args);

    // DBSCAN 主函数
    static int dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, double eps, int min_pts);

private:
    // DBSCAN 嵌套类
    class DBSCAN{
    public:
        DBSCAN(double eps, int min_pts) : eps_(eps), min_pts_(min_pts) {};
        int run(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud);
    private:
        double eps_;
        int min_pts_;

        std::vector<int> regionQuery(const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, const pcl::PointXYZL& point);
        void labelPoint(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, const std::vector<int>& points, int label);
        int expandCluster(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, int point_idx, int cluster_id);
        void removeNoisePoints(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud);
    };
};