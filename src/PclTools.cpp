#include "PclTools.hpp"

// ��ʱ����ģ��ʵ��
template<typename Func, typename... Args>
auto PCLTools::timeFunction(Func&& func, Args&&... args){
    auto start = std::chrono::high_resolution_clock::now();
    auto result = std::forward<Func>(func)(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "ִ��ʱ��: " << duration.count() << " ����" << std::endl;
    return result;
}

// DBSCAN ������ʵ��
int PCLTools::dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, double eps, int min_pts) {
    DBSCAN dbscan(eps, min_pts);
    return dbscan.run(cloud);
}

// DBSCAN ���к���ʵ��
int PCLTools::DBSCAN::run(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud) {
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    kdtree.setInputCloud(cloud);

    int cluster_id = 0;
    int num_clusters = 0;

    // ��ʼ����ǩ
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
        cloud->points[i].label = 0;
    }

    // ����
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

    // �Ƴ�������
    removeNoisePoints(cloud);
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    return num_clusters;
}

// �����ѯ����ʵ��
std::vector<int> PCLTools::DBSCAN::regionQuery(const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, const pcl::PointXYZL& point) {
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(point, eps_, neighbors, pointRadiusSquaredDistance);
    return neighbors;
}

// ��ǵ㺯��ʵ��
void PCLTools::DBSCAN::labelPoint(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, const std::vector<int>& points, int label) {
    for (int idx : points) {
        cloud->points[idx].label = label;
    }
}

// ��չ�غ���ʵ��
int PCLTools::DBSCAN::expandCluster(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, int point_idx, int cluster_id) {
    std::vector<int> seeds = regionQuery(cloud, kdtree, cloud->points[point_idx]);
    if (seeds.size() < min_pts_) {
        cloud->points[point_idx].label = 4294967290; // ���Ϊ������
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
                if (cloud->points[idx].label == 0) { // ������δ��ǵĵ�
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

// �Ƴ������㺯��ʵ��
void PCLTools::DBSCAN::removeNoisePoints(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    cleaned_cloud->points.reserve(cloud->points.size());

#pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr local_cleaned(new pcl::PointCloud<pcl::PointXYZL>);

#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
            if (cloud->points[i].label != 4294967290) { // ����������
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