#include <pcl/io/pcd_io.h>
#include <omp.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <vector> 
#include <iostream> 
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
 
 
std::vector<int> region_query(const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, const pcl::PointXYZL& point, double eps) {
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(point, eps, neighbors, pointRadiusSquaredDistance);
    return neighbors;
}
 
void label_point(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, const std::vector<int>& points, int label) {
    for (int idx : points) {
        cloud->points[idx].label = label;
    }
}
 
int expand_cluster(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, int point_idx, int cluster_id, double eps, int min_pts) {
    std::vector<int> seeds = region_query(cloud, kdtree, cloud->points[point_idx], eps);
    if (seeds.size() < min_pts) {
        cloud->points[point_idx].label = 4294967290; // 将噪声点标记为4294967290
        return 0;
    }
 
    label_point(cloud, seeds, cluster_id);
    int cluster_size = seeds.size();
 
    while (!seeds.empty()) {
        int current_point = seeds.back();
        seeds.pop_back();
 
        std::vector<int> result = region_query(cloud, kdtree, cloud->points[current_point], eps);
        if (result.size() >= min_pts) {
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
 
    // std::cout << "簇 " << cluster_id << " 的点数为 " << cluster_size << std::endl;
    return cluster_size;
}
 
void remove_noise_points(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    cleaned_cloud->points.reserve(cloud->points.size()); // 预分配内存，提升性能
 
#pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr local_cleaned(new pcl::PointCloud<pcl::PointXYZL>);
 
#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
            if (cloud->points[i].label != 4294967290) { // 忽略标记为噪声的点
                local_cleaned->points.push_back(cloud->points[i]);
            }
        }
 
#pragma omp critical
        {
            cleaned_cloud->points.insert(cleaned_cloud->points.end(), local_cleaned->points.begin(), local_cleaned->points.end());
        }
    }
 
    cloud.swap(cleaned_cloud); // 更新原始点云
}
 
void dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, double eps, int min_pts, int& num_clusters) {
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    kdtree.setInputCloud(cloud);
 
    int cluster_id = 0;
    int local_num_clusters = 0;
 
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
        cloud->points[i].label = 0;
    }
 
#pragma omp parallel for schedule(dynamic) reduction(+:local_num_clusters)
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
        if (cloud->points[i].label == 0) {
            int cluster_size = expand_cluster(cloud, kdtree, i, cluster_id + 1, eps, min_pts);
            if (cluster_size > 0) {
#pragma omp critical
                {
                    ++cluster_id;
                }
                ++local_num_clusters;
            }
        }
    }
    num_clusters += local_num_clusters;
 
    remove_noise_points(cloud);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    // std::cout << "簇的数量为：" << num_clusters << std::endl;
}