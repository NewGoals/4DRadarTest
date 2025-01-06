#include "PclTools.hpp"

// 计时函数模版实现
template<typename Func, typename... Args>
auto PCLTools::timeFunction(Func&& func, Args&&... args){
    auto start = std::chrono::high_resolution_clock::now();
    auto result = std::forward<Func>(func)(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "执行时间: " << duration.count() << " 毫秒" << std::endl;
    return result;
}

// DBSCAN 主函数实现
int PCLTools::dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, double eps, int min_pts) {
    DBSCAN dbscan(eps, min_pts);
    return dbscan.run(cloud);
}

// DBSCAN 运行函数实现
int PCLTools::DBSCAN::run(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud) {
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    kdtree.setInputCloud(cloud);

    int cluster_id = 0;
    int num_clusters = 0;

    // 初始化标签
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
        cloud->points[i].label = 0;
    }

    // 聚类
#pragma omp parallel for schedule(dynamic) reduction(+:num_clusters)
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
        if (cloud->points[i].label == 0) {
            int cluster_size = expandCluster(cloud, kdtree, i, cluster_id + 1);
            if (cluster_size > 0) {
#pragma omp critical
                {
                    ++cluster_id;
                }
                ++num_clusters;
            }
        }
    }

    // 移除噪声点
    removeNoisePoints(cloud);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return num_clusters;
}

// 区域查询函数实现
std::vector<int> PCLTools::DBSCAN::regionQuery(const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, const pcl::PointXYZL& point) {
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(point, eps_, neighbors, pointRadiusSquaredDistance);
    return neighbors;
}

// 标记点函数实现
void PCLTools::DBSCAN::labelPoint(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, const std::vector<int>& points, int label) {
    for (int idx : points) {
        cloud->points[idx].label = label;
    }
}

// 扩展簇函数实现
int PCLTools::DBSCAN::expandCluster(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, int point_idx, int cluster_id) {
    std::vector<int> seeds = regionQuery(cloud, kdtree, cloud->points[point_idx]);
    if (seeds.size() < min_pts_) {
        cloud->points[point_idx].label = 4294967290; // 标记为噪声点
        return 0;
    }

    labelPoint(cloud, seeds, cluster_id);
    int cluster_size = seeds.size();

    while (!seeds.empty()) {
        int current_point = seeds.back();
        seeds.pop_back();

        std::vector<int> result = regionQuery(cloud, kdtree, cloud->points[current_point]);
        if (result.size() >= min_pts_) {
            for (int idx : result) {
                if (cloud->points[idx].label == 0) { // 仅处理未标记的点
                    bool push = false;
#pragma omp critical
                    {
                        if (cloud->points[idx].label == 0) {
                            cloud->points[idx].label = cluster_id;
                            cluster_size++;
                            push = true;
                        }
                    }
                    if (push) {
                        seeds.push_back(idx);
                    }
                }
            }
        }
    }

    return cluster_size;
}

// 移除噪声点函数实现
void PCLTools::DBSCAN::removeNoisePoints(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    cleaned_cloud->points.reserve(cloud->points.size());

#pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr local_cleaned(new pcl::PointCloud<pcl::PointXYZL>);

#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
            if (cloud->points[i].label != 4294967290) { // 忽略噪声点
                local_cleaned->points.push_back(cloud->points[i]);
            }
        }

#pragma omp critical
        {
            cleaned_cloud->points.insert(cleaned_cloud->points.end(), local_cleaned->points.begin(), local_cleaned->points.end());
        }
    }

    cloud.swap(cleaned_cloud);
}