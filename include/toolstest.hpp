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
