// RadarAlertSystem.cpp
#include "RadarAlertSystem.hpp"

RadarAlertSystem::RadarAlertSystem(const RadarAlertConfig &config)
    : config_(config), cloud_(new pcl::PointCloud<pcl::PointXYZ>),
      kdtree_(new pcl::KdTreeFLANN<pcl::PointXYZ>)
{
    cloud_->header.frame_id = "world";
    cloud_->height = 1;
}

RadarAlertSystem::~RadarAlertSystem()
{
    running_.store(false); // 关闭主线程，原子变量在多线程环境下，也不会出现数据竞争
}

// 防区管理方法实现
void RadarAlertSystem::addDefenseZone(const DefenseZone &zone)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    defense_zones_.push_back({zone, false, 0.0f, {}});
}

void RadarAlertSystem::updateDefenseZone(int zone_id, const DefenseZone &new_config)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto &zone_status : defense_zones_)
    {
        if (zone_status.config.zone_id == zone_id)
        {
            zone_status.config = new_config;
            break;
        }
    }
}

void RadarAlertSystem::removeDefenseZone(int zone_id)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    defense_zones_.erase(
        std::remove_if(
            defense_zones_.begin(),
            defense_zones_.end(),
            [zone_id](const ZoneStatus &z)
            {
                return z.config.zone_id == zone_id;
            }),
        defense_zones_.end());
}

void RadarAlertSystem::clearAllZones()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    defense_zones_.clear();
}

/// @brief 处理流程：1. 将原始点转换为vector<ProcessedPoint>结构；2. 将点加入pcl kdtree；3. 判断是否在防区内。
/// @param raw_points
/// @param velocities
/// @param timestamp
void RadarAlertSystem::processFrame(const std::vector<Eigen::Vector3f> &raw_points,
                                    const std::vector<float> &velocities,
                                    double timestamp)
{
    std::vector<ProcessedPoint> processed;
    transformCoordinates(processed, raw_points, velocities, timestamp);
    // 加锁
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        updateSpatialIndex(processed);
        checkZones(processed);
    }
}

void RadarAlertSystem::setAlertCallback(const AlertCallback &callback)
{
    alert_callback_ = callback;
}

std::vector<RadarAlertSystem::DefenseZone> RadarAlertSystem::getDefenseZones()
{
    std::vector<DefenseZone> zones;
    {
        std::lock_guard<std::mutex> lock(data_mutex_); // 保证线程安全

        zones.reserve(defense_zones_.size());
        for (const auto &zone_status : defense_zones_)
        {
            zones.push_back(zone_status.config);
        }
    }
    return zones;
}

void RadarAlertSystem::transformCoordinates(std::vector<ProcessedPoint> &output,
                                            const std::vector<Eigen::Vector3f> &input,
                                            const std::vector<float> &velocities,
                                            double timestamp)
{
    output.reserve(input.size());
    for (size_t i = 0; i < input.size(); ++i)
    {
        Eigen::Vector4f homog(input[i].x(), input[i].y(), input[i].z(), 1.0f);
        Eigen::Vector4f transformed = config_.radar_to_world * homog;

        if (transformed.head<3>().norm() <= config_.max_detection_range)
        {
            output.emplace_back(ProcessedPoint{
                transformed.head<3>(),
                velocities[i],
                timestamp});
        }
    }
}

void RadarAlertSystem::updateSpatialIndex(const std::vector<ProcessedPoint> &points)
{
    cloud_->clear();
    cloud_->width = points.size();
    cloud_->points.resize(points.size());

#pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i)
    {
        cloud_->points[i].x = points[i].position.x();
        cloud_->points[i].y = points[i].position.y();
        cloud_->points[i].z = points[i].position.z();
    }

    kdtree_->setInputCloud(cloud_);
}

void RadarAlertSystem::checkZones(const std::vector<ProcessedPoint> &points)
{
    for (auto &zone : defense_zones_)
    {
        int valid_count = 0; // 有效点计算
        std::vector<Eigen::Vector3f> triggers;

        if (zone.config.shape == ZoneType::CUBOID || zone.config.shape == ZoneType::ROTATED_CUBOID)
        {

            // 使用AABB进行空间查询，kdtree_->boxSearch可能有点问题，手动循环实现
            // 手动遍历所有点
            for (size_t idx = 0; idx < points.size(); ++idx)
            {
                const auto &pt = points[idx];
                const Eigen::Vector3f &pos = pt.position;

                // 检查点是否在AABB范围内
                if (std::abs(pt.velocity) > zone.config.min_speed &&
                    std::abs(pt.velocity) < zone.config.max_speed &&
                    isInZone(zone, pos))
                {
                    valid_count++;
                    triggers.push_back(pos);
                }
            }
        }
        else
        {
            std::cout << "未定义的防区类型" << std::endl;
        }

        // 持续时间计算, 动态衰减策略
        const float delta = config_.update_interval;
        if (valid_count >= zone.config.min_points)
        {
            zone.duration += delta; // 有目标时正常增加持续时间
        }
        else
        {
            // 没有目标时，若持续时长远大于阈值则进行动态衰减
            if (zone.duration > zone.config.trigger_duration + 2)
            {
                // 计算每帧需要的衰减量
                float decay_amount = (zone.duration - zone.config.trigger_duration - 2) * 0.7f;
                // 确保衰减量合理
                decay_amount = std::max(decay_amount, delta); // 最大衰减量为 delta * 2
                zone.duration = std::max(0.0f, zone.duration - decay_amount);
            }
            else
            {
                // 如果 duration 已经低于触发阈值，按默认衰减因子衰减
                zone.duration = std::max(0.0f, zone.duration - delta);
            }
        }

        // std::cout << "valid count: " << valid_count << std::endl;
        // std::cout << zone.duration << std::endl;

        // 状态判断和回调（保持不变）
        const bool new_status = zone.duration >= zone.config.trigger_duration;
        if (new_status != zone.is_alert)
        {
            if (new_status)
                zone.duration += 2;     // 触发一次报警，则延长duration时长防止在阈值附近反复报警
            zone.is_alert = new_status;
            zone.trigger_points = triggers;

            if (alert_callback_)
            {
                alert_callback_(zone.config.zone_id, new_status);
            }
        }
    }
}

bool RadarAlertSystem::isInZone(const ZoneStatus &zone, const Eigen::Vector3f &point) const
{
    switch (zone.config.shape)
    {
    case ZoneType::CUBOID:
    {
        const auto &min = zone.config.params[0];
        const auto &max = zone.config.params[1];
        return (point.x() >= min.x() && point.x() <= max.x() &&
                point.y() >= min.y() && point.y() <= max.y() &&
                point.z() >= min.z() && point.z() <= max.z());
    }
    case ZoneType::CYLINDER:
    {
        const auto &center = zone.config.params[0];
        const auto &radius = zone.config.params[1].x();
        const auto &height = zone.config.params[1].y();
        const float dx = point.x() - center.x();
        const float dy = point.y() - center.y();
        return (dx * dx + dy * dy <= radius * radius) &&
               (point.z() >= center.z() && point.z() <= center.z() + height);
    }
    case ZoneType::POLYGON:
    {
        // 使用射线法实现多边形检测
        const size_t n = zone.config.params.size();
        bool inside = false;
        for (size_t i = 0, j = n - 1; i < n; j = i++)
        {
            const auto &vi = zone.config.params[i];
            const auto &vj = zone.config.params[j];
            if (((vi.y() > point.y()) != (vj.y() > point.y())) &&
                (point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) / (vj.y() - vi.y()) + vi.x()))
            {
                inside = !inside;
            }
        }
        return inside;
    }
    case ZoneType::ROTATED_CUBOID:
    {
        const auto &center = zone.config.params[0];    // 中心点
        const auto &size = zone.config.params[1];      // 尺寸（长、宽、高）
        const float angle = zone.config.params[2].x(); // 旋转角度（弧度）

        // 将点从世界坐标系转换到 Box 的局部坐标系
        Eigen::Vector3f local_point = point - center;

        // 旋转点（绕 Z 轴）
        float cos_angle = std::cos(angle);
        float sin_angle = std::sin(angle);
        float x = local_point.x() * cos_angle + local_point.y() * sin_angle;
        float y = -local_point.x() * sin_angle + local_point.y() * cos_angle;
        local_point.x() = x;
        local_point.y() = y;

        // 在局部坐标系中检测 AABB
        return (local_point.x() >= -size.x() / 2 && local_point.x() <= size.x() / 2 &&
                local_point.y() >= -size.y() / 2 && local_point.y() <= size.y() / 2 &&
                local_point.z() >= -size.z() / 2 && local_point.z() <= size.z() / 2);
    }
    // case ZoneType::ROTATED_CUBOID_3DOF:
    // {
    //     const auto &center = zone.config.params[0]; // 中心点
    //     const auto &size = zone.config.params[1];   // 尺寸（长、宽、高）
    //     const auto &angles = zone.config.params[2]; // 旋转角度（绕 X、Y、Z 轴）

    //     // 将点从世界坐标系转换到 Box 的局部坐标系
    //     Eigen::Vector3f local_point = point - center;

    //     // 构建绕 X、Y、Z 轴的旋转矩阵
    //     Eigen::Quaternionf quaternion =
    //         Eigen::AngleAxisf(angles.x(), Eigen::Vector3f::UnitX()) *
    //         Eigen::AngleAxisf(angles.y(), Eigen::Vector3f::UnitY()) *
    //         Eigen::AngleAxisf(angles.z(), Eigen::Vector3f::UnitZ());

    //     Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();

    //     // 旋转点
    //     local_point = rotation_matrix * local_point;

    //     // 在局部坐标系中检测 AABB
    //     return (local_point.x() >= -size.x() / 2 && local_point.x() <= size.x() / 2 &&
    //             local_point.y() >= -size.y() / 2 && local_point.y() <= size.y() / 2 &&
    //             local_point.z() >= -size.z() / 2 && local_point.z() <= size.z() / 2);
    // }
    default:
        return false;
    }
}


//==============================================================================
// 轨迹预警策略
//==============================================================================
RadarTraceAlertSystem::RadarTraceAlertSystem(const RadarTraceAlertConfig &config)
    : config_(config), last_cleanup_time_(0)
{
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
}

RadarTraceAlertSystem::~RadarTraceAlertSystem()
{
    running_ = false;
}

void RadarTraceAlertSystem::addDefenseZone(const DefenseZone &zone)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    ZoneStatus new_zone;
    new_zone.config = zone;
    defense_zones_.push_back(new_zone);
}

void RadarTraceAlertSystem::updateDefenseZone(int zone_id, const DefenseZone &new_config)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto &zone : defense_zones_)
    {
        if (zone.config.zone_id == zone_id)
        {
            zone.config = new_config;
            return;
        }
    }
}

void RadarTraceAlertSystem::removeDefenseZone(int zone_id)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = std::remove_if(defense_zones_.begin(), defense_zones_.end(),
                            [zone_id](const ZoneStatus &zone) {
                                return zone.config.zone_id == zone_id;
                            });
    
    if (it != defense_zones_.end())
    {
        defense_zones_.erase(it, defense_zones_.end());
    }
}

void RadarTraceAlertSystem::clearAllZones()
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    defense_zones_.clear();
}

void RadarTraceAlertSystem::processFrame(const RadarTraceData &trace_data)
{
    if (!running_)
        return;
    
    double current_time = trace_data.timestamp;
    
    // 转换雷达轨迹到世界坐标系
    std::vector<ProcessedTrace> processed_traces = transformTraces(trace_data);
    
    // 更新轨迹历史记录
    updateTraceHistory(processed_traces, current_time);
    
    // 更新空间索引，用于可视化
    updateSpatialIndex(processed_traces);
    
    // 检查防区
    checkZones(current_time);
    
    // 清理过期轨迹 (每秒进行一次清理)
    if (current_time - last_cleanup_time_ > 1.0)
    {
        cleanupExpiredTraces(current_time);
        last_cleanup_time_ = current_time;
    }
}

std::vector<RadarTraceAlertSystem::ProcessedTrace> RadarTraceAlertSystem::transformTraces(const RadarTraceData &trace_data)
{
    std::vector<ProcessedTrace> processed_traces;
    processed_traces.reserve(trace_data.traces.size());
    
    for (const auto &trace : trace_data.traces)
    {
        // 距离过滤
        if (trace.length < config_.min_detection_range || trace.length > config_.max_detection_range)
            continue;
        
        // 转换到世界坐标系
        Eigen::Vector4f point_radar(trace.x_axes, trace.y_axes, trace.z_axes, 1.0f);
        Eigen::Vector4f point_world = config_.radar_to_world * point_radar;
        
        Eigen::Vector4f vel_radar(trace.x_speed, trace.y_speed, trace.z_speed, 0.0f);
        Eigen::Vector4f vel_world = config_.radar_to_world * vel_radar;
        
        ProcessedTrace pt;
        pt.ID = trace.ID;
        pt.position = Eigen::Vector3f(point_world.x(), point_world.y(), point_world.z());
        pt.velocity = Eigen::Vector3f(vel_world.x(), vel_world.y(), vel_world.z());
        pt.timestamp = trace_data.timestamp;
        pt.SNR = trace.SNR;
        pt.length = trace.length;
        
        processed_traces.push_back(pt);
    }
    
    return processed_traces;
}

void RadarTraceAlertSystem::updateTraceHistory(const std::vector<ProcessedTrace> &traces, double current_time)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 标记当前帧中所有出现的轨迹ID
    std::set<uint32_t> current_ids;
    for (const auto &trace : traces)
    {
        current_ids.insert(trace.ID);
    }
    
    // 更新历史记录
    for (const auto &trace : traces)
    {
        auto it = trace_history_.find(trace.ID);
        
        if (it == trace_history_.end())
        {
            // 新轨迹
            TraceHistory history;
            history.trace_id = trace.ID;
            history.positions.push_back(trace);
            history.continuous_frames = 1;
            history.last_update_time = current_time;
            trace_history_[trace.ID] = history;
        }
        else
        {
            // 已有轨迹
            it->second.positions.push_back(trace);
            it->second.continuous_frames++;
            it->second.last_update_time = current_time;
            
            // 限制历史记录长度，只保留最近的N个位置
            if (it->second.positions.size() > 30) // 可配置
            {
                it->second.positions.pop_front();
            }
        }
    }
    
    // 对于当前帧中没有出现的轨迹，重置连续计数
    for (auto &history_pair : trace_history_)
    {
        if (current_ids.find(history_pair.first) == current_ids.end())
        {
            history_pair.second.continuous_frames = 0;
        }
    }
}

void RadarTraceAlertSystem::updateSpatialIndex(const std::vector<ProcessedTrace> &traces)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 更新点云用于可视化
    cloud_->clear();
    cloud_->points.reserve(traces.size());
    
    for (const auto &trace : traces)
    {
        pcl::PointXYZ pt;
        pt.x = trace.position.x();
        pt.y = trace.position.y();
        pt.z = trace.position.z();
        cloud_->points.push_back(pt);
    }
    
    cloud_->width = cloud_->points.size();
    cloud_->height = 1;
    cloud_->is_dense = true;
    
    // 更新KD树(暂不更新)
    // kdtree_->setInputCloud(cloud_);
}

void RadarTraceAlertSystem::checkZones(double current_time)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 清空每个防区当前的轨迹ID列表
    for (auto &zone : defense_zones_)
    {
        zone.current_trace_ids.clear();
    }
    
    // 检查每个轨迹和防区的关系
    for (auto &trace_pair : trace_history_)
    {
        auto &history = trace_pair.second;
        
        // 只处理连续跟踪达到阈值的轨迹
        if (history.continuous_frames < config_.min_continuous_frames)
            continue;
        
        // 获取最新位置
        if (history.positions.empty())
            continue;
            
        const auto &latest_pos = history.positions.back();
        
        // 速度检查（可选）
        float speed = latest_pos.velocity.norm();
        
        // 检查每个防区
        for (auto &zone : defense_zones_)
        {
            // 速度过滤
            if (speed < zone.config.min_speed || speed > zone.config.max_speed)
                continue;
                
            // 检查最新位置是否在防区内
            if (isInZone(zone, latest_pos.position))
            {
                // 添加到当前防区的轨迹ID列表
                zone.current_trace_ids.push_back(history.trace_id);
                
                // 设置告警状态
                bool prev_status = zone.alert_traces[history.trace_id];
                zone.alert_traces[history.trace_id] = true;
                
                // 检查防区整体告警状态
                bool new_zone_alert = !zone.current_trace_ids.empty();
                
                // 如果状态改变，触发回调
                if (prev_status != true || zone.is_alert != new_zone_alert) 
                {
                    if (alert_callback_)
                    {
                        alert_callback_(zone.config.zone_id, true, history.trace_id);
                    }
                }
                
                zone.is_alert = new_zone_alert;
            }
            else
            {
                // 检查是否需要取消告警
                auto alert_it = zone.alert_traces.find(history.trace_id);
                if (alert_it != zone.alert_traces.end() && alert_it->second)
                {
                    alert_it->second = false;
                    
                    // 触发取消告警回调
                    if (alert_callback_)
                    {
                        alert_callback_(zone.config.zone_id, false, history.trace_id);
                    }
                    
                    // 检查是否所有轨迹都已离开防区
                    bool any_alert = false;
                    for (const auto &trace_alert : zone.alert_traces)
                    {
                        if (trace_alert.second)
                        {
                            any_alert = true;
                            break;
                        }
                    }
                    
                    zone.is_alert = any_alert;
                }
            }
        }
    }
}

bool RadarTraceAlertSystem::isInZone(const ZoneStatus &zone, const Eigen::Vector3f &point) const
{
    switch (zone.config.shape)
    {
        case ZoneType::CUBOID:
        {
            // 立方体防区，参数：[min_x, min_y, min_z, max_x, max_y, max_z]
            if (zone.config.params.size() < 2)
                return false;
                
            const auto &min_point = zone.config.params[0];
            const auto &max_point = zone.config.params[1];
            
            return (point.x() >= min_point.x() && point.x() <= max_point.x() &&
                    point.y() >= min_point.y() && point.y() <= max_point.y() &&
                    point.z() >= min_point.z() && point.z() <= max_point.z());
        }
        
        case ZoneType::CYLINDER:
        {
            // 圆柱防区，参数：[center_x, center_y, center_z, radius, height]
            if (zone.config.params.size() < 2)
                return false;
                
            const auto &center = zone.config.params[0];
            float radius = zone.config.params[1].x();
            float height = zone.config.params[1].y();
            
            float dx = point.x() - center.x();
            float dy = point.y() - center.y();
            float distance_2d = std::sqrt(dx * dx + dy * dy);
            
            return (distance_2d <= radius &&
                    point.z() >= center.z() &&
                    point.z() <= center.z() + height);
        }
        
        case ZoneType::POLYGON:
        {
            // 多边形防区（2D），参数：按顺序的多边形顶点，z为高度范围
            if (zone.config.params.size() < 3)
                return false;
                
            // 射线法检查点是否在多边形内
            bool inside = false;
            float min_z = zone.config.params[0].z();
            float max_z = zone.config.params[1].z();
            
            // 首先检查z轴
            if (point.z() < min_z || point.z() > max_z)
                return false;
            
            // 然后检查点是否在多边形内部（射线法）
            size_t n_vertices = zone.config.params.size() - 2;
            for (size_t i = 2, j = n_vertices + 1; i <= n_vertices + 1; j = i++)
            {
                const auto &vi = zone.config.params[i];
                const auto &vj = zone.config.params[j];
                
                if (((vi.y() > point.y()) != (vj.y() > point.y())) &&
                    (point.x() < (vj.x() - vi.x()) * (point.y() - vi.y()) / (vj.y() - vi.y()) + vi.x()))
                {
                    inside = !inside;
                }
            }
            
            return inside;
        }
        
        case ZoneType::ROTATED_CUBOID:
        {
            // 旋转立方体，参数：[center_x, center_y, center_z, size_x, size_y, size_z, rotation_z]
            if (zone.config.params.size() < 3)
                return false;
                
            const auto &center = zone.config.params[0];
            const auto &size = zone.config.params[1];
            float rotation_z = zone.config.params[2].x();
            
            // 将点转换到局部坐标系
            Eigen::Vector3f local_point = point - center;
            
            // 应用旋转（绕Z轴）
            float cos_rot = std::cos(rotation_z);
            float sin_rot = std::sin(rotation_z);
            
            Eigen::Vector3f rotated_point;
            rotated_point.x() = local_point.x() * cos_rot + local_point.y() * sin_rot;
            rotated_point.y() = -local_point.x() * sin_rot + local_point.y() * cos_rot;
            rotated_point.z() = local_point.z();
            
            // 检查是否在立方体内部
            return (std::abs(rotated_point.x()) <= size.x() / 2 &&
                    std::abs(rotated_point.y()) <= size.y() / 2 &&
                    std::abs(rotated_point.z()) <= size.z() / 2);
        }
        
        case ZoneType::ROTATED_CUBOID_3DOF:
        {
            // 三自由度旋转立方体，参数：[center, size, rotation(roll, pitch, yaw)]
            if (zone.config.params.size() < 3)
                return false;
                
            const auto &center = zone.config.params[0];
            const auto &size = zone.config.params[1];
            const auto &rotation = zone.config.params[2];
            
            // 将点转换到局部坐标系
            Eigen::Vector3f local_point = point - center;
            
            // 计算旋转矩阵（Roll-Pitch-Yaw顺序）
            float cr = std::cos(rotation.x());
            float sr = std::sin(rotation.x());
            float cp = std::cos(rotation.y());
            float sp = std::sin(rotation.y());
            float cy = std::cos(rotation.z());
            float sy = std::sin(rotation.z());
            
            Eigen::Matrix3f rotation_matrix;
            rotation_matrix << 
                cp*cy, cy*sr*sp - cr*sy, sr*sy + cr*cy*sp,
                cp*sy, cr*cy + sr*sp*sy, cr*sp*sy - cy*sr,
                -sp, cp*sr, cp*cr;
            
            // 应用旋转
            Eigen::Vector3f rotated_point = rotation_matrix.transpose() * local_point;
            
            // 检查是否在立方体内部
            return (std::abs(rotated_point.x()) <= size.x() / 2 &&
                    std::abs(rotated_point.y()) <= size.y() / 2 &&
                    std::abs(rotated_point.z()) <= size.z() / 2);
        }
        
        default:
            return false;
    }
}


void RadarTraceAlertSystem::cleanupExpiredTraces(double current_time)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    const double expiration_time = 3.0; // 3秒不更新的轨迹将被删除
    
    auto it = trace_history_.begin();
    while (it != trace_history_.end())
    {
        if (current_time - it->second.last_update_time > expiration_time)
        {
            // 清理对应的防区告警
            for (auto &zone : defense_zones_)
            {
                auto alert_it = zone.alert_traces.find(it->first);
                if (alert_it != zone.alert_traces.end() && alert_it->second)
                {
                    alert_it->second = false;
                    
                    // 触发回调
                    if (alert_callback_)
                    {
                        alert_callback_(zone.config.zone_id, false, it->first);
                    }
                }
                
                // 从当前ID列表中移除
                auto id_it = std::find(zone.current_trace_ids.begin(), zone.current_trace_ids.end(), it->first);
                if (id_it != zone.current_trace_ids.end())
                {
                    zone.current_trace_ids.erase(id_it);
                }
                
                // 更新防区整体告警状态
                bool any_alert = false;
                for (const auto &trace_alert : zone.alert_traces)
                {
                    if (trace_alert.second)
                    {
                        any_alert = true;
                        break;
                    }
                }
                
                zone.is_alert = any_alert;
            }
            
            // 删除轨迹历史
            it = trace_history_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

const pcl::PointCloud<pcl::PointXYZ>::ConstPtr RadarTraceAlertSystem::getVisualizationCloud() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    return cloud_;
}

void RadarTraceAlertSystem::getAlertStatus(std::map<int, std::vector<uint32_t>> &status_map) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    status_map.clear();
    
    for (const auto &zone : defense_zones_)
    {
        if (zone.is_alert)
        {
            std::vector<uint32_t> alert_ids;
            for (const auto &trace_alert : zone.alert_traces)
            {
                if (trace_alert.second)
                {
                    alert_ids.push_back(trace_alert.first);
                }
            }
            status_map[zone.config.zone_id] = alert_ids;
        }
    }
}

void RadarTraceAlertSystem::setAlertCallback(const AlertCallback &callback)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    alert_callback_ = callback;
}

std::vector<RadarTraceAlertSystem::DefenseZone> RadarTraceAlertSystem::getDefenseZones() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<DefenseZone> zones;
    zones.reserve(defense_zones_.size());
    
    for (const auto &zone : defense_zones_)
    {
        zones.push_back(zone.config);
    }
    
    return zones;
}