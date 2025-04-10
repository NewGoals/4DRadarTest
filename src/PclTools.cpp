#include "PclTools.hpp"

// 计时函数模版实现
template <typename Func, typename... Args>
auto PCLTools::timeFunction(Func &&func, Args &&...args)
{
    auto start = std::chrono::high_resolution_clock::now();
    auto result = std::forward<Func>(func)(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "执行时间: " << duration.count() << " 毫秒" << std::endl;
    return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLTools::Raw2PointRGB_vr(const std::shared_ptr<RadarData> &radar_data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    float MAX_ABS_SPEED = 10.0f;
    for (const auto &point : radar_data->points)
    {
        pcl::PointXYZRGB p_rcs;
        p_rcs.x = point.x;
        p_rcs.y = point.y;
        p_rcs.z = point.z;
        // 计算颜色（HSV -> RGB）
        float speed = point.v_r;

        float abs_speed = std::min(std::abs(speed), MAX_ABS_SPEED);
        float hue = 0.0;

        // 颜色映射规则
        if (speed < 0)
        {
            // 正速度：红(0°) -> 黄(60°)
            hue = 60.0f - 60.0f * (abs_speed / MAX_ABS_SPEED);
        }
        else if (speed > 0)
        {
            // 负速度：蓝(240°) -> 青(180°)
            hue = 180.0f + 60.0f * (abs_speed / MAX_ABS_SPEED);
        }
        else
        {
            hue = 120.0f;
        }

        // 转换HSV到RGB
        cv::Mat hsv(1, 1, CV_32FC3, cv::Scalar(hue, 1.0, 1.0));
        cv::Mat bgr;
        cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);

        // 设置点云颜色
        p_rcs.r = static_cast<uint8_t>(bgr.at<cv::Vec3f>(0, 0)[2] * 255);
        p_rcs.g = static_cast<uint8_t>(bgr.at<cv::Vec3f>(0, 0)[1] * 255);
        p_rcs.b = static_cast<uint8_t>(bgr.at<cv::Vec3f>(0, 0)[0] * 255);
        source_cloud->push_back(p_rcs);
    }
    return source_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLTools::RadarTraceData2PointRGB(const std::shared_ptr<RadarTraceData> &radar_trace_data)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (!radar_trace_data->traces.size())
        // std::cout << "radar_trace_data empty! " << std::endl;
        return cloud;

    // 类型颜色映射表（BGR格式）
    static const std::unordered_map<int, cv::Vec3b> TYPE_COLORS = {
        {1, {0, 255, 0}},   // 小动物 - 绿色
        {2, {0, 255, 255}}, // 人    - 黄色
        {4, {0, 0, 255}}    // 车    - 红色
    };
    cloud->resize(radar_trace_data->traces.size());

    for (const auto &trace : radar_trace_data->traces)
    {
        // 查找颜色映射
        auto it = TYPE_COLORS.find(trace.type);
        if (it == TYPE_COLORS.end())
            continue;

        pcl::PointXYZRGB point;
        point.x = trace.x_axes;
        point.y = trace.y_axes;
        point.z = trace.z_axes;

        // 设置颜色（PCL使用RGB顺序）
        const auto &color = it->second;
        point.r = color[2];
        point.g = color[1];
        point.b = color[0];

        cloud->push_back(point);
    }
    return cloud;
}

void PCLTools::PointViewerInit(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    // ========== 网格参数配置 ==========
    const float y_start = 0.0f;    // Y轴起始坐标
    const float y_end = 200.0f;    // Y轴结束坐标
    const float x_min = -20.0f;    // X轴最小坐标
    const float x_max = 20.0f;     // X轴最大坐标
    const float grid_size = 10.0f; // 网格边长
    const float z_level = 0.0f;    // Z轴固定高度

    // ========== 文字标签参数 ==========
    const float text_offset_x = 21.0f; // 文本X轴偏移（左侧外扩1米）
    const float text_size = 3.0;       // 字体大小
    const double text_r = 1.0;         // 文字颜色（白色）
    const double text_g = 1.0;
    const double text_b = 1.0;

    // ========== 网格线样式配置 ==========
    const std::string grid_id_prefix = "grid_line_"; // 网格线ID前缀
    const float line_width = 0.5f;                   // 网格线宽度
    const double r = 0.5, g = 0.5, b = 0.5;          // 网格线颜色（灰色）

    // ========== 生成横向网格线（沿X轴方向） ==========
    for (float y = y_start; y <= y_end; y += grid_size)
    {
        // 生成线条起止点
        pcl::PointXYZ start(x_min, y, z_level);
        pcl::PointXYZ end(x_max, y, z_level);

        // 生成唯一ID
        std::stringstream ss;
        ss << grid_id_prefix << "x_" << y;

        // 添加线条到可视化器
        viewer->addLine<pcl::PointXYZ>(start, end, r, g, b, ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                            line_width, ss.str());
    }

    // ========== 生成纵向网格线（沿Y轴方向） ==========
    for (float x = x_min; x <= x_max; x += grid_size)
    {
        // 生成线条起止点
        pcl::PointXYZ start(x, y_start, z_level);
        pcl::PointXYZ end(x, y_end, z_level);

        // 生成唯一ID
        std::stringstream ss;
        ss << grid_id_prefix << "y_" << x;

        // 添加线条到可视化器
        viewer->addLine<pcl::PointXYZ>(start, end, r, g, b, ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                            line_width, ss.str());
    }

    // ========== 沿Y轴添加50米间隔文字标签 ==========
    for (float y = y_start; y <= y_end; y += 50.0f)
    {
        // 生成文本内容（例如 "50m"）
        std::stringstream label_ss;
        label_ss << static_cast<int>(y) << "m";

        // 创建 pcl::PointXYZ 类型的点（关键修改）
        pcl::PointXYZ text_pos;
        text_pos.x = text_offset_x; // X坐标偏移
        text_pos.y = y;             // Y坐标
        text_pos.z = z_level;       // Z坐标

        // 生成唯一ID
        std::string text_id = "y_label_" + std::to_string(static_cast<int>(y));

        // 添加文字标签
        viewer->addText3D( // 使用 pcl::PointXYZ 而非 Eigen::Vector3f
            label_ss.str(),
            text_pos, // 参数类型修正
            text_size,
            text_r, text_g, text_b,
            text_id);
    }

    // ========== 设置俯视视角 ==========
    const float camera_height = 300.0f;   // 相机高度（Z轴位置）
    const float lookat_center_y = 100.0f; // 观察焦点 Y 轴中心（0-200范围的中点）

    viewer->setCameraPosition(
        0.0,             // 相机X位置（居中）
        lookat_center_y, // 相机Y位置（居中）
        camera_height,   // 相机Z位置（高空俯视）
        0.0,             // 焦点X位置
        lookat_center_y, // 焦点Y位置
        0.0,             // 焦点Z位置（网格所在平面）
        0, 1, 0          // 上方向向量（Y轴方向保证正北朝上）
    );

    viewer->setCameraFieldOfView(0.8); // 减小视野角度，增强俯视正交感

    // ========== 边界加强线 ==========
    // 绘制外围边界（可选，增强边界可见性）
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x_min, y_start, z_level),
                                   pcl::PointXYZ(x_min, y_end, z_level),
                                   1.0, 0.0, 0.0, "left_boundary");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x_max, y_start, z_level),
                                   pcl::PointXYZ(x_max, y_end, z_level),
                                   1.0, 0.0, 0.0, "right_boundary");
    viewer->addLine<pcl::PointXYZ>(pcl::PointXYZ(x_min, y_end, z_level),
                                   pcl::PointXYZ(x_max, y_end, z_level),
                                   1.0, 0.0, 0.0, "top_boundary");
}

void PCLTools::RenderAccumulatedData(pcl::visualization::PCLVisualizer::Ptr viewer, const std::deque<std::shared_ptr<RadarTraceData>> &data_sequence, size_t latest_frames)
{
    viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    // 仅处理最近N帧
    size_t start_idx = data_sequence.size() > latest_frames ? data_sequence.size() - latest_frames : 0;
    // std::cout << "star_idx: " << start_idx << "data_sequence:" << data_sequence.size() << std::endl;

    // 轨迹线ID映射表
    std::unordered_map<int, pcl::PointXYZ> id_position_map;

    // 提取首帧ID集合 --------------------------------------------------
    std::unordered_set<int> first_frame_ids;
    if (!data_sequence.empty() && start_idx < data_sequence.size())
    {
        const auto &first_frame = data_sequence[start_idx];
        for (const auto &trace : first_frame->traces)
        {
            first_frame_ids.insert(trace.ID);
            // std::cout << "first id: " << trace.ID << std::endl;
        }
    }

    for (size_t i = start_idx; i < data_sequence.size(); ++i)
    {
        const auto &frame = data_sequence[i];
        auto cloud = RadarTraceData2PointRGB(frame);

        // 动态透明度：越旧的数据越透明
        float alpha = 0.3f + 0.7f * (i - start_idx) / latest_frames;

        // 添加点云
        std::string cloud_name = "frame_" + std::to_string(i);
        viewer->addPointCloud(cloud, cloud_name);

        // 设置显示属性
        // viewer->setPointCloudRenderingProperties(
        //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        //     i == data_sequence.size() - 1 ? 3.0f : 1.0f, // 最新帧点更大
        //     cloud_name);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            1.0f, // 最新帧点更大
            cloud_name);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY,
            alpha,
            cloud_name);

        // 构建轨迹连线，过滤虚警
        for (const auto &trace : frame->traces)
        {
            // if (id_position_map.count(trace.ID))
            // {
            //     const auto &prev_point = id_position_map[trace.ID];
            //     pcl::PointXYZ curr_point(trace.x_axes, trace.y_axes, trace.z_axes);

            //     // 添加线段
            //     viewer->addLine(prev_point, curr_point, 0, 1.0, 0,
            //                     "line_" + std::to_string(trace.ID) + "_" + std::to_string(frame->timestamp));
            // }

            // id_position_map[trace.ID] = pcl::PointXYZ(
            //     trace.x_axes, trace.y_axes, trace.z_axes);

            if (first_frame_ids.count(trace.ID) && i == data_sequence.size() - 1)
            {
                // std::cout << "exsit point" << std::endl;
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                    3.0f, // 最新帧点更大
                    cloud_name);
            }
        }
    }
}

void PCLTools::addDefenseZoneVisualization(pcl::visualization::PCLVisualizer::Ptr viewer,
                                           const RadarTraceAlertSystem::DefenseZone &zone)
{
    switch (zone.shape)
    {
    case RadarTraceAlertSystem::ZoneType::CUBOID:
    {
        if (zone.params.size() >= 2)
        {
            const auto &min_point = zone.params[0];
            const auto &max_point = zone.params[1];

            // 创建立方体表示
            std::string cube_id = "zone_" + std::to_string(zone.zone_id);
            pcl::PointXYZ p1(min_point.x(), min_point.y(), min_point.z());
            pcl::PointXYZ p2(max_point.x(), max_point.y(), max_point.z());
            viewer->addCube(p1.x, p2.x, p1.y, p2.y, p1.z, p2.z, 0.0, 1.0, 0.0, cube_id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                cube_id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, cube_id);
        }
        break;
    }
    // 可以继续添加其他形状的可视化...

    default:
        break;
    }
}

void PCLTools::updateAlertVisualization(pcl::visualization::PCLVisualizer::Ptr viewer,
                                        const RadarTraceAlertSystem &alert_system,
                                        const std::map<int, std::vector<uint32_t>> &alert_status)
{
    // 获取所有防区配置
    auto zones = alert_system.getDefenseZones();
    
    // 更新防区颜色
    for (const auto& zone : zones)
    {
        std::string zone_id = "zone_" + std::to_string(zone.zone_id);

        // 检查形状是否存在
        if (!viewer->contains(zone_id))
        {
            // 如果形状不存在，重新添加防区
            addDefenseZoneVisualization(viewer, zone);
            continue;
        }
        
        bool is_alert = alert_status.find(zone.zone_id) != alert_status.end();
        
        // 如果防区处于告警状态，设置为红色；否则设置为绿色
        if (is_alert)
        {
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, zone_id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, zone_id);
        }
        else
        {
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, zone_id);
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, zone_id);
        }
    }
    
    // 获取告警系统的可视化点云（包含所有当前的轨迹点）
    auto viz_cloud = alert_system.getVisualizationCloud();
    static bool first_update = true;
    
    if (viz_cloud && !viz_cloud->empty())
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        // 转换普通点云为带颜色的点云
        colored_cloud->points.resize(viz_cloud->points.size());
        colored_cloud->width = viz_cloud->width;
        colored_cloud->height = viz_cloud->height;
        
        // 根据点是否在告警区域内设置不同颜色
        for (size_t i = 0; i < viz_cloud->points.size(); ++i)
        {
            colored_cloud->points[i].x = viz_cloud->points[i].x;
            colored_cloud->points[i].y = viz_cloud->points[i].y;
            colored_cloud->points[i].z = viz_cloud->points[i].z;
            
            // 默认为白色
            colored_cloud->points[i].r = 255;
            colored_cloud->points[i].g = 255; 
            colored_cloud->points[i].b = 255;
            
            // 目前我们没有直接的方法知道哪个点对应哪个轨迹ID
            // 所以这里简单地将所有点设置为白色
        }
        
        // 更新或添加点云
        if (first_update)
        {
            viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "alert_points");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "alert_points");
            first_update = false;
        }
        else
        {
            viewer->updatePointCloud<pcl::PointXYZRGB>(colored_cloud, "alert_points");
        }
    }
}

// DBSCAN 主函数实现
int PCLTools::dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, double eps, int min_pts)
{
    DBSCAN dbscan(eps, min_pts);
    return dbscan.run(cloud);
}

// DBSCAN 运行函数实现
int PCLTools::DBSCAN::run(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud)
{
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    kdtree.setInputCloud(cloud);

    int cluster_id = 0;
    int num_clusters = 0;

    // 初始化标签
#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
    {
        cloud->points[i].label = 0;
    }

    // 聚类
#pragma omp parallel for schedule(dynamic) reduction(+ : num_clusters)
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
    {
        if (cloud->points[i].label == 0)
        {
            int cluster_size = expandCluster(cloud, kdtree, i, cluster_id + 1);
            if (cluster_size > 0)
            {
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
std::vector<int> PCLTools::DBSCAN::regionQuery(const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, pcl::KdTreeFLANN<pcl::PointXYZL> &kdtree, const pcl::PointXYZL &point)
{
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    kdtree.radiusSearch(point, eps_, neighbors, pointRadiusSquaredDistance);
    return neighbors;
}

// 标记点函数实现
void PCLTools::DBSCAN::labelPoint(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, const std::vector<int> &points, int label)
{
    for (int idx : points)
    {
        cloud->points[idx].label = label;
    }
}

// 扩展簇函数实现
int PCLTools::DBSCAN::expandCluster(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, pcl::KdTreeFLANN<pcl::PointXYZL> &kdtree, int point_idx, int cluster_id)
{
    std::vector<int> seeds = regionQuery(cloud, kdtree, cloud->points[point_idx]);
    if (seeds.size() < min_pts_)
    {
        cloud->points[point_idx].label = 4294967290; // 标记为噪声点
        return 0;
    }

    labelPoint(cloud, seeds, cluster_id);
    int cluster_size = seeds.size();

    while (!seeds.empty())
    {
        int current_point = seeds.back();
        seeds.pop_back();

        std::vector<int> result = regionQuery(cloud, kdtree, cloud->points[current_point]);
        if (result.size() >= min_pts_)
        {
            for (int idx : result)
            {
                if (cloud->points[idx].label == 0)
                { // 仅处理未标记的点
                    bool push = false;
#pragma omp critical
                    {
                        if (cloud->points[idx].label == 0)
                        {
                            cloud->points[idx].label = cluster_id;
                            cluster_size++;
                            push = true;
                        }
                    }
                    if (push)
                    {
                        seeds.push_back(idx);
                    }
                }
            }
        }
    }

    return cluster_size;
}

// 移除噪声点函数实现
void PCLTools::DBSCAN::removeNoisePoints(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZL>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    cleaned_cloud->points.reserve(cloud->points.size());

#pragma omp parallel
    {
        pcl::PointCloud<pcl::PointXYZL>::Ptr local_cleaned(new pcl::PointCloud<pcl::PointXYZL>);

#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
        {
            if (cloud->points[i].label != 4294967290)
            { // 忽略噪声点
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