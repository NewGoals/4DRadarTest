// #include <pcl/io/pcd_io.h>
// #include <omp.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <vector>
// #include <iostream>
// #include <pcl/search/kdtree.h>
// #include <pcl/common/common.h>

// std::vector<int> region_query(const pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, const pcl::PointXYZL& point, double eps) {
//     std::vector<int> neighbors;
//     std::vector<float> pointRadiusSquaredDistance;
//     kdtree.radiusSearch(point, eps, neighbors, pointRadiusSquaredDistance);
//     return neighbors;
// }

// void label_point(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, const std::vector<int>& points, int label) {
//     for (int idx : points) {
//         cloud->points[idx].label = label;
//     }
// }

// int expand_cluster(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, pcl::KdTreeFLANN<pcl::PointXYZL>& kdtree, int point_idx, int cluster_id, double eps, int min_pts) {
//     std::vector<int> seeds = region_query(cloud, kdtree, cloud->points[point_idx], eps);
//     if (seeds.size() < min_pts) {
//         cloud->points[point_idx].label = 4294967290; // 将噪声点标记为4294967290
//         return 0;
//     }

//     label_point(cloud, seeds, cluster_id);
//     int cluster_size = seeds.size();

//     while (!seeds.empty()) {
//         int current_point = seeds.back();
//         seeds.pop_back();

//         std::vector<int> result = region_query(cloud, kdtree, cloud->points[current_point], eps);
//         if (result.size() >= min_pts) {
//             for (int idx : result) {
//                 if (cloud->points[idx].label == 0) { // 仅处理未标记的点
//                     bool push = false;
// #pragma omp critical
//                     {
//                         if (cloud->points[idx].label == 0) {
//                             cloud->points[idx].label = cluster_id;
//                             cluster_size++;
//                             push = true;
//                         }
//                     }
//                     if (push) {
//                         seeds.push_back(idx);
//                     }
//                 }
//             }
//         }
//     }

//     // std::cout << "簇 " << cluster_id << " 的点数为 " << cluster_size << std::endl;
//     return cluster_size;
// }

// void remove_noise_points(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud) {
//     pcl::PointCloud<pcl::PointXYZL>::Ptr cleaned_cloud(new pcl::PointCloud<pcl::PointXYZL>);
//     cleaned_cloud->points.reserve(cloud->points.size()); // 预分配内存，提升性能

// #pragma omp parallel
//     {
//         pcl::PointCloud<pcl::PointXYZL>::Ptr local_cleaned(new pcl::PointCloud<pcl::PointXYZL>);

// #pragma omp for nowait
//         for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
//             if (cloud->points[i].label != 4294967290) { // 忽略标记为噪声的点
//                 local_cleaned->points.push_back(cloud->points[i]);
//             }
//         }

// #pragma omp critical
//         {
//             cleaned_cloud->points.insert(cleaned_cloud->points.end(), local_cleaned->points.begin(), local_cleaned->points.end());
//         }
//     }

//     cloud.swap(cleaned_cloud); // 更新原始点云
// }

// void dbscan(pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud, double eps, int min_pts, int& num_clusters) {
//     pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
//     kdtree.setInputCloud(cloud);

//     int cluster_id = 0;
//     int local_num_clusters = 0;

// #pragma omp parallel for
//     for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
//         cloud->points[i].label = 0;
//     }

// #pragma omp parallel for schedule(dynamic) reduction(+:local_num_clusters)
//     for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
//         if (cloud->points[i].label == 0) {
//             int cluster_size = expand_cluster(cloud, kdtree, i, cluster_id + 1, eps, min_pts);
//             if (cluster_size > 0) {
// #pragma omp critical
//                 {
//                     ++cluster_id;
//                 }
//                 ++local_num_clusters;
//             }
//         }
//     }
//     num_clusters += local_num_clusters;

//     remove_noise_points(cloud);
//     cloud->width = cloud->points.size();
//     cloud->height = 1;
//     cloud->is_dense = true;
//     // std::cout << "簇的数量为：" << num_clusters << std::endl;
// }

// #include "DataReaderFactory.hpp"
// #include <filesystem>
// namespace fs = std::filesystem;

// void addRadarFrame(std::string base_path, int num){
//     // 从文件中读取数据
//     std::string radar_path = base_path + "/radar";
//     auto radar_reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE, radar_path);
//     if (!radar_reader->init()) {
//         std::cerr << "初始化读取器失败" << std::endl;
//         return;
//     }

//     // 提取文件夹路径（去掉文件名部分）
//     std::string folder_path = base_path + "/" + std::to_string(num) + "frame_radar";

//     // 检查文件夹是否存在，如果不存在则创建它
//     if (!fs::exists(folder_path)) {
//         try {
//             fs::create_directories(folder_path);
//         } catch (const std::exception& e) {
//             std::cerr << "无法创建文件夹: " << folder_path << ", 错误: " << e.what() << std::endl;
//             return;  // 如果无法创建文件夹，则终止程序
//         }
//     }

//     std::deque<std::vector<TargetInfoParse_0xA8::TargetInfo>> sliding_window; // 滑动窗口
//     int frame_count = 0;

//     while (!radar_reader->isEnd()) {
//         std::vector<TargetInfoParse_0xA8::TargetInfo> targets = std::dynamic_pointer_cast<RadarFileReader>(radar_reader)->readNext_0xA8();
//         if (!targets.empty()) {
//             if (sliding_window.size() >= num) {
//                 sliding_window.pop_front(); // 移除最旧的一帧
//             }
//             sliding_window.push_back(targets); // 加入新的一帧
//             frame_count++;

//             if (sliding_window.size() <= num && sliding_window.size() > 0) {
//                 // 合并滑动窗口中的所有帧
//                 std::vector<TargetInfoParse_0xA8::TargetInfo> merged_targets;
//                 for (const auto& frame : sliding_window) {
//                     merged_targets.insert(merged_targets.end(), frame.begin(), frame.end());
//                 }

//                 // 将合并后的数据保存为二进制文件
//                 std::string output_path = folder_path + "/merged_radar_frame_" + std::to_string(frame_count - 1) + ".bin";
//                 std::ofstream out_file(output_path, std::ios::binary);
//                 if (!out_file) {
//                     std::cerr << "无法创建输出文件: " << output_path << std::endl;
//                     continue;
//                 }

//                 // 写入点的数量
//                 uint32_t num_points = static_cast<uint32_t>(merged_targets.size());
//                 out_file.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

//                 // 写入每个点的信息
//                 // 一次性写入所有目标数据
//                 out_file.write(reinterpret_cast<const char*>(merged_targets.data()), merged_targets.size() * sizeof(TargetInfoParse_0xA8::TargetInfo));

//                 out_file.close();
//                 // std::cout << "数据已成功保存到 " << output_path << std::endl;
//             }
//         }

//         // 处理滑动窗口中剩余的帧（不满num帧的情况）
//         // if (!sliding_window.empty()) {
//         //     // 合并滑动窗口中的所有帧
//         //     std::vector<TargetInfoParse_0xA8::TargetInfo> merged_targets;
//         //     for (const auto& frame : sliding_window) {
//         //         merged_targets.insert(merged_targets.end(), frame.begin(), frame.end());
//         //     }

//         //     // 将合并后的数据保存为二进制文件
//         //     std::string output_path = base_path + "/merged_radar_frame_" + std::to_string(frame_count - sliding_window.size() + 1) + ".bin";
//         //     std::ofstream out_file(output_path, std::ios::binary);
//         //     if (!out_file) {
//         //         std::cerr << "无法创建输出文件: " << output_path << std::endl;
//         //         return;
//         //     }

//         //     // 写入点的数量
//         //     size_t num_points = merged_targets.size();
//         //     out_file.write(reinterpret_cast<const char*>(&num_points), sizeof(num_points));

//         //     // 写入每个点的信息
//         //     for (const auto& target : merged_targets) {
//         //         out_file.write(reinterpret_cast<const char*>(&target), sizeof(target));
//         //     }

//         //     out_file.close();
//         //     std::cout << "数据已成功保存到 " << output_path << std::endl;
//         // }
//     }
// }

// #include <onnxruntime_cxx_api.h>
// #include <opencv2/opencv.hpp>
// #include <vector>

// void testyolov6Onnx(){
//     // 1. 加载模型
//     Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "YOLOv6");
//     Ort::SessionOptions session_options;

//     Ort::Session session(env, L"E:/Source/4DRadarTest/models/last_ckpt.onnx", session_options); // 注意 L 表示宽字符（Windows路径）

//     // 2. 图像预处理
//     cv::Mat image = cv::imread("E:/dataset/manualCalib/camera_near/camera_near_193546.jpg");
//     if (image.empty()) {
//         std::cerr << "Error: Could not load image!" << std::endl;
//         return;
//     }

//     // Letterbox处理 ----------------------------------------------------
//     const int target_height = 640;
//     const int target_width = 640;
//     cv::Scalar pad_color(114, 114, 114);

//     // 计算缩放比例
//     cv::Size orig_size = image.size();
//     float r = std::min(float(target_height)/orig_size.height,
//                     float(target_width)/orig_size.width);
//     cv::Size new_unpad(int(round(orig_size.width * r)),
//                     int(round(orig_size.height * r)));

//     // 调整尺寸
//     cv::Mat resized;
//     if (orig_size != new_unpad) {
//         cv::resize(image, resized, new_unpad, 0, 0, cv::INTER_LINEAR);
//     } else {
//         resized = image.clone();
//     }

//     // 计算填充
//     int dw = target_width - new_unpad.width;
//     int dh = target_height - new_unpad.height;
//     int top = dh / 2;
//     int bottom = dh - top;
//     int left = dw / 2;
//     int right = dw - left;

//     // 添加边框
//     cv::copyMakeBorder(resized, resized, top, bottom, left, right,
//                     cv::BORDER_CONSTANT, pad_color);

//     // 保存缩放和填充参数供后处理使用
//     float ratio = r;
//     float pad_w = left;
//     float pad_h = top;

//     // 图像预处理 -------------------------------------------------------
//     // BGR -> RGB
//     cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);

//     // 转换为CHW格式 (HWC -> CHW)
//     std::vector<cv::Mat> rgb_channels;
//     cv::split(resized, rgb_channels);
//     std::vector<float> input_tensor_values;
//     for (auto& c : rgb_channels) {
//         cv::Mat channel_float;
//         c.convertTo(channel_float, CV_32F, 1.0 / 255.0);
//         input_tensor_values.insert(input_tensor_values.end(),
//             channel_float.begin<float>(), channel_float.end<float>());
//     }

//     // 3. 准备输入张量
//     Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
//     std::vector<int64_t> input_shape = {1, 3, 640, 640};
//     Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
//         memory_info, input_tensor_values.data(), input_tensor_values.size(),
//         input_shape.data(), input_shape.size()
//     );

//     // 4. 运行推理
//     std::vector<const char*> input_names = {"images"};
//     std::vector<const char*> output_names = {"outputs"};
//     auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_names.data(), &input_tensor, 1, output_names.data(), 1);
//     float* output_data = output_tensors[0].GetTensorMutableData<float>();

//     // 5. 后处理（示例代码需根据模型输出调整）
//     const std::vector<std::string> CLASS_NAMES = {"person", "car"};
//     float confidence_threshold = 0.6; // 提高置信度阈值
//     float nms_threshold = 0.3;        // 降低 IoU 阈值
//     const int num_classes = 2;
//     const int data_per_box = 5 + num_classes;

//     std::vector<cv::Rect> boxes;
//     std::vector<float> scores;
//     std::vector<int> class_ids;

//     // 输出格式为 [1, 8400, 7]
//     for (int i = 0; i < 8400; ++i) {
//         float* ptr = output_data + i * data_per_box;
//         float obj_score = ptr[4];                // 目标存在置信度
//         float* cls_scores = ptr + 5;             // 类别概率起始位置
//         int class_id = std::max_element(cls_scores, cls_scores + num_classes) - cls_scores;
//         float cls_score = cls_scores[class_id];  // 最大类别概率
//         float final_score = obj_score * cls_score;  // 最终置信度

//         if (final_score > confidence_threshold) {
//             int class_id = std::max_element(ptr + 5, ptr + data_per_box) - (ptr + 5);
//             // 坐标转换（假设模型输出归一化坐标）
//             // 修改后的坐标转换（使用保存的缩放和填充参数）
//             float x_center = (ptr[0] - pad_w) / ratio;  // 转换到原始图像坐标系
//             float y_center = (ptr[1] - pad_h) / ratio;
//             float width = ptr[2] / ratio;
//             float height = ptr[3] / ratio;

//             cv::Rect box(
//                 static_cast<int>(x_center - width / 2),
//                 static_cast<int>(y_center - height / 2),
//                 static_cast<int>(width),
//                 static_cast<int>(height)
//             );

//             // 约束坐标范围
//             box.x = std::max(0, std::min(box.x, image.cols - 1));
//             box.y = std::max(0, std::min(box.y, image.rows - 1));
//             box.width = std::max(1, std::min(box.width, image.cols - box.x));
//             box.height = std::max(1, std::min(box.height, image.rows - box.y));

//             boxes.push_back(box);
//             scores.push_back(final_score);
//             class_ids.push_back(class_id);
//         }
//     }

//     // 按类别分组应用 NMS
//     std::map<int, std::vector<cv::Rect>> class_boxes;
//     std::map<int, std::vector<float>> class_scores;
//     for (size_t i = 0; i < boxes.size(); ++i) {
//         class_boxes[class_ids[i]].push_back(boxes[i]);
//         class_scores[class_ids[i]].push_back(scores[i]);
//     }

//     std::vector<cv::Rect> final_boxes;
//     std::vector<int> final_class_ids;
//     for (auto& pair : class_boxes) {
//         int class_id = pair.first;
//         std::vector<cv::Rect> cls_boxes = pair.second;
//         std::vector<float> cls_scores = class_scores[class_id];

//         std::vector<int> indices;
//         cv::dnn::NMSBoxes(cls_boxes, cls_scores, confidence_threshold, nms_threshold, indices);

//         for (int idx : indices) {
//             final_boxes.push_back(cls_boxes[idx]);
//             final_class_ids.push_back(class_id);
//         }
//     }

//     // 绘制最终结果
//     for (size_t i = 0; i < final_boxes.size(); ++i) {
//         cv::Rect box = final_boxes[i];
//         cv::rectangle(image, box, cv::Scalar(0, 255, 0), 2);
//         std::string label = cv::format("%s: %.2f", CLASS_NAMES[final_class_ids[i]].c_str(), scores[i]);
//         cv::putText(image, label, cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
//     }

//     cv::imshow("Result", image);
//     cv::waitKey(0);
// }

#include "RadarAlertSystem.hpp"
#include "ManualCalib.hpp"

void testRadarAlert()
{
    // ---------------------- 1. 初始化配置 ----------------------
    // 加载雷达坐标系到世界坐标系的标定参数
    RadarImageMapper mapper;
    mapper.init("camera_calibration.yml", "E:/Source/4DRadarTest/extrinsic_calibration.yml");

    // 雷达报警系统配置
    RadarAlertConfig alert_config;
    alert_config.radar_to_world = Eigen::Matrix4f::Identity(); // 若需要，从标定参数转换
    alert_config.max_detection_range = 200.0f;                 // 最大检测距离200米
    alert_config.velocity_filter_thresh = 0.5f;                // 速度波动阈值0.5m/s
    alert_config.update_interval = 0.1f;                       // 每0.1秒更新一次

    // 创建雷达报警系统实例
    RadarAlertSystem radar_alert_system(alert_config);

    // ---------------------- 2. 配置防区 ----------------------
    // 添加立方体防区 (ID=1)
    RadarAlertSystem::DefenseZone cuboid_zone{
        1,
        RadarAlertSystem::ZoneType::CUBOID,
        {Eigen::Vector3f(-4, 15, -2), Eigen::Vector3f(4, alert_config.max_detection_range, 10)},
        0.001f,
        1000.0f,
        3,
        0.8f};
    radar_alert_system.addDefenseZone(cuboid_zone);

    // 设置报警回调
    radar_alert_system.setAlertCallback([](int zone_id, bool status) { // 获取当前时间（含毫秒）
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

        // 格式化为字符串
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
            << '.' << std::setfill('0') << std::setw(3) << ms.count();

        // 输出带时间的报警信息
        std::cout << "[" << oss.str() << "] ALERT: Zone " << zone_id
                  << (status ? " TRIGGERED!" : " Cleared!")
                  << std::endl;
    });

    // ---------------------- 2.5 点云融合配置 ----------------------
    const bool enable_fusion = true;  // 控制是否启用融合
    const int fusion_window_size = 5; // 滑动窗口大小（3帧）
    std::deque<std::vector<RadarPoint>> fusion_buffer;

    // ---------------------- 3. 初始化数据读取 ----------------------
    bool online = true;

    // 数据初始化
    auto radar_reader = DataReaderFactory::createReader(ReaderType::RADAR_FILE,
                                                        "E:/dataset/test/radar");
    SynchronizedCollector collector;
    std::cout << "创建采集器成功" << std::endl;
    if (!online)
    {
        if (!radar_reader->init())
        {
            std::cerr << "雷达数据读取器初始化失败!" << std::endl;
            return;
        }
        std::cout << "离线数据初始化成功, 开始离线读取数据!" << std::endl;
    }
    else
    {
        // 实时数据初始化
        // 添加主数据源（雷达）
        auto radarSource = std::make_unique<RadarSource>("192.168.88.219", 12345, "radar");
        if (!radarSource->init())
        {
            throw std::runtime_error("雷达初始化失败");
        }
        collector.addSource(std::move(radarSource), true);
        std::cout << "添加雷达源成功" << std::endl;
        auto videoSource_near = std::make_unique<VideoSource>("rtsp://192.168.88.219:554/live/chn1/stream_1", "camera_near");
        if (!videoSource_near->init())
        {
            throw std::runtime_error("视频源1初始化失败");
        }
        collector.addSource(std::move(videoSource_near), false);
        std::cout << "添加视频源成功" << std::endl;

        collector.setSaveConfig(true, true, RadarFileReader::Format::BIN);
        collector.start();
        std::cout << "实时数据初始化成功, 开始在线读取数据!" << std::endl;
    }

    // ---------------------- 4. 可视化初始化 ----------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Radar Alert Visualization"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0); // 显示坐标系

    // ---------------------- 5. 多帧处理循环 ----------------------
    double timestamp = 0.0;
    while (true)
    {
        // ---------------------- (1). 数据处理部分 ----------------------
        std::shared_ptr<RadarData> radar_data = nullptr;
        // 读取雷达数据
        if (!online)
        {
            if (radar_reader->isEnd())
                break;
            radar_reader->readNext();
            radar_data = std::dynamic_pointer_cast<RadarData>(radar_reader->getData());
        }
        else
        {
            radar_data = collector.getMainSourceData();
        }

        // 判断是否进行点云融合显示
        if (enable_fusion)
        {
            fusion_buffer.push_back(radar_data->points);
            // 维护窗口大小
            while (fusion_buffer.size() > fusion_window_size)
            {
                fusion_buffer.pop_front();
            }
            // 简单合并
            std::vector<RadarPoint> temp_points;
            for (const auto &frame : fusion_buffer)
            {
                temp_points.insert(temp_points.end(), frame.begin(), frame.end());
            }
            radar_data->points = temp_points;
            // std::cout << "point size = " << temp_points.size() << std::endl;
        }

        // 提取点云和速度
        std::vector<Eigen::Vector3f> points;
        std::vector<float> velocities;
        for (const auto &point : radar_data->points)
        {
            points.emplace_back(point.x, point.y, point.z);
            velocities.push_back(point.v_r); // 假设v_r为径向速度
        }

        // 处理当前帧
        radar_alert_system.processFrame(points, velocities, timestamp);

        // ---------------------- (2). 结果渲染部分 ----------------------
        // 清除上一帧的可视化内容
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // 可视化原始点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        PCLTools pcltools;
        cloud = pcltools.Raw2PointRGB_vr(radar_data);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "raw_points");

        // 可视化防区边界
        for (const auto &zone : radar_alert_system.getDefenseZones())
        { // 假设有getDefenseZones()
            if (zone.shape == RadarAlertSystem::ZoneType::CUBOID)
            {
                Eigen::Vector3f min_pt = zone.params[0];
                Eigen::Vector3f max_pt = zone.params[1];
                viewer->addCube(
                    min_pt.x(), max_pt.x(),
                    min_pt.y(), max_pt.y(),
                    min_pt.z(), max_pt.z(),
                    1.0, 0.0, 0.0, // 红色边框
                    "zone_" + std::to_string(zone.zone_id));
                viewer->setShapeRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3,
                    "zone_" + std::to_string(zone.zone_id));
            }
        }

        auto subData = collector.getSubSourceData();
        for (const auto &data : subData)
        {
            auto imageData = std::make_shared<ImageData>();
            imageData->frame = mapper.cubeBottomMapper(data.second, -4, 15, -2, 4, 200, 15, -1.5);
            imshow("Frame near", imageData->frame);
        }

        // 触发渲染
        viewer->spinOnce(10);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 模拟实时
        timestamp += alert_config.update_interval;
    }
}

void testTraceData()
{
    // 设置雷达模式
    int modeType = 0;
    PCLTools pcltools;

    // 添加主数据源（雷达）
    SynchronizedCollector collector;
    std::cout << "创建采集器成功" << std::endl;
    auto radarSource = std::make_unique<RadarSource>("192.168.88.219", 12345, "radar");
    if (!radarSource->init())
    {
        throw std::runtime_error("雷达初始化失败");
    }
    radarSource->setModeType(modeType);
    collector.addSource(std::move(radarSource), true);
    std::cout << "添加雷达源成功" << std::endl;

    collector.setSaveConfig(false, false, RadarFileReader::Format::BIN);
    collector.start();

    std::cout << "实时数据初始化成功, 开始在线读取数据!" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Radar Alert Visualization"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0); // 显示坐标系
    pcltools.PointViewerInit(viewer);

    // 在循环外部声明持久化变量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_valid_cloud(nullptr);
    auto last_render_time = std::chrono::steady_clock::now();

    std::deque<std::shared_ptr<RadarTraceData>> history_data;
    while (true)
    {
        try
        {
            // 限流控制：最高30FPS渲染
            constexpr double MAX_FPS = 30.0;
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_render_time)
                    .count() < 1000.0 / MAX_FPS)
            {
                Sleep(1);
                continue;
            }

            auto radar_trace_data = collector.getMainSourceTraceDataFromBuffer();

            if (radar_trace_data)
            {
                history_data.push_back(radar_trace_data);
                if (history_data.size() > 10)
                { // 限制最大缓存量防止内存溢出
                    history_data.pop_front();
                }
            }

            collector.printStats();

            // 始终用最后有效数据渲染
            // if (last_valid_cloud && !last_valid_cloud->empty())
            // {
            //     viewer->updatePointCloud(last_valid_cloud, "raw_points");
            // }
            pcltools.RenderAccumulatedData(viewer, history_data, 10);

            // 触发渲染
            viewer->spinOnce(10);
            last_render_time = std::chrono::steady_clock::now();
        }
        catch (const std::exception &e)
        {
            // 处理循环内的局部异常，避免程序退出
            std::cerr << "数据渲染异常: " << e.what() << std::endl;
            collector.stop(); // 停止采集器
            viewer->close();  // 关闭可视化窗口
            break;            // 退出循环
        }
    }
}

void testTraceAlertSystem()
{
    // 设置雷达模式
    int modeType = 0; // 安防模式
    PCLTools pcltools;

    // 添加主数据源（雷达）
    SynchronizedCollector collector;
    std::cout << "创建采集器成功" << std::endl;
    auto radarSource = std::make_unique<RadarSource>("192.168.88.219", 12345, "radar");
    // auto radarSource = std::make_unique<RadarSource>("192.168.10.117", 50000, "radar");
    if (!radarSource->init())
    {
        throw std::runtime_error("雷达初始化失败");
    }
    radarSource->setModeType(modeType);
    collector.addSource(std::move(radarSource), true);
    std::cout << "添加雷达源成功" << std::endl;

    collector.setSaveConfig(false, false, RadarFileReader::Format::BIN);
    collector.start();

    std::cout << "实时数据初始化成功, 开始在线读取数据!" << std::endl;

    // 初始化雷达告警系统
    RadarTraceAlertConfig alert_config;
    // 设置雷达坐标系到世界坐标系的转换矩阵
    alert_config.radar_to_world = Eigen::Matrix4f::Identity(); // 根据实际情况调整
    alert_config.min_continuous_frames = 10;                   // 设置需要连续跟踪10帧才触发告警
    alert_config.max_detection_range = 250.0f;
    alert_config.min_detection_range = 0.0f;

    RadarTraceAlertSystem alert_system(alert_config);

    // 添加防区
    RadarTraceAlertSystem::DefenseZone zone1;
    zone1.zone_id = 1;
    zone1.shape = RadarTraceAlertSystem::ZoneType::CUBOID;
    // 设置防区范围 [min_x, min_y, min_z, max_x, max_y, max_z]
    zone1.params.push_back(Eigen::Vector3f(-10.0f, 0.0f, -5.0f));  // 最小点
    zone1.params.push_back(Eigen::Vector3f(10.0f, 200.0f, 10.0f)); // 最大点
    zone1.min_speed = 0.0f;                                        // 最小速度阈值，单位 m/s
    zone1.max_speed = 30.0f;                                       // 最大速度阈值
    zone1.min_points = 1;                                          // 最小点数
    alert_system.addDefenseZone(zone1);

    // 设置告警回调函数
    alert_system.setAlertCallback([](int zone_id, bool status, uint32_t trace_id)
                                  {
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 转换为 time_t 类型（C风格时间）
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        // 转换为本地时间（线程不安全版本，需注意多线程环境）
        std::tm* local_time = std::localtime(&now_time); 
        // 格式化输出时间，线程安全版需要改进
        std::cout << "[" << std::put_time(local_time, "%Y-%m-%d %H:%M:%S") << "] ";

        if (status) {
            std::cout << "警告! 防区 " << zone_id << " 检测到目标 ID: " << trace_id << std::endl;
        } else {
            std::cout << "目标 ID: " << trace_id << " 已离开防区 " << zone_id << std::endl;
        } });

    // 初始化PCL可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Radar Alert Visualization"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0); // 显示坐标系
    pcltools.PointViewerInit(viewer);

    // 在循环外部声明持久化变量
    auto last_render_time = std::chrono::steady_clock::now();

    // 告警状态显示
    std::map<int, std::vector<uint32_t>> alert_status;

    std::deque<std::shared_ptr<RadarTraceData>> history_data;

    // 在循环外部定义帧率和计时变量（建议放在合适的作用域内）
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true)
    {
        try
        {
            // // 限流控制：最高30FPS渲染
            // constexpr double MAX_FPS = 30.0;
            // if (std::chrono::duration_cast<std::chrono::milliseconds>(
            //         std::chrono::steady_clock::now() - last_render_time)
            //         .count() < 1000.0 / MAX_FPS)
            // {
            //     Sleep(1);
            //     continue;
            // }

            auto render_start = std::chrono::high_resolution_clock::now(); // 记录开始时间
            auto radar_trace_data = collector.getMainSourceTraceDataFromBuffer();
            // auto radar_trace_data = collector.getMainSourceTraceData();

            if (radar_trace_data->traces.size())
            {
                // std::cout << "size: " << radar_trace_data->traces.size() << ", timestramp: " << radar_trace_data->timestamp << std::endl;
                // std::cout << "current time: " << collector.getCurrentTimestamp() << std::endl;
                // 使用告警系统处理当前帧
                alert_system.processFrame(*radar_trace_data);

                // 获取当前告警状态并显示
                alert_system.getAlertStatus(alert_status);

                // 保存历史数据用于可视化
                history_data.push_back(radar_trace_data);
                if (history_data.size() > 20)
                { // 限制最大缓存量防止内存溢出
                    history_data.pop_front();
                }

                // std::cout << "history size: " << history_data.size() << std::endl;

                // collector.printStats();

                // 更新告警系统可视化
                pcltools.updateAlertVisualization(viewer, alert_system, alert_status);

                // 渲染雷达点
                pcltools.RenderAccumulatedData(viewer, history_data, 20);

                frame_count++;
            }
            else{
                std::cout << "radar trace empty!" << std::endl;
            }

            // 帧率计算逻辑（每次循环都会执行）
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

            // 每1秒计算一次实际帧率
            if (elapsed >= 5000)
            {
                double fps = static_cast<double>(frame_count) / (elapsed / 5000.0);
                std::cout << "[帧率统计] 取数帧率: " << fps << " FPS" << std::endl;

                // 重置计数器和时间戳
                frame_count = 0;
                start_time = current_time;
            }

            // 触发渲染
            viewer->spinOnce(10);
            last_render_time = std::chrono::steady_clock::now();
            Sleep(10);
        }
        catch (const std::exception &e)
        {
            // 处理循环内的局部异常，避免程序退出
            std::cerr << "数据渲染异常: " << e.what() << std::endl;
            collector.stop(); // 停止采集器
            viewer->close();  // 关闭可视化窗口
            break;            // 退出循环
        }
    }
}
