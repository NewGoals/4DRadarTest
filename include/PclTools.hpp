#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>

#include <memory>
#include <functional>
#include <vector>

// namespace PCLTools {
// // 自定义点类型，与radardata保持一致
// struct RadarPointPCL 
// {
//     PCL_ADD_POINT4D;  // x, y, z
//     float rcs;        // 雷达截面积
//     float v_r;        // 径向速度
    
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// // 注册点类型
// POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointPCL,
//     (float, x, x)
//     (float, y, y)
//     (float, z, z)
//     (float, rcs, rcs)
//     (float, v_r, v_r)
// )

// }