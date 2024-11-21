cmake_minimum_required(VERSION 3.10)

project(4DRadarTest)

# 添加这一行来解决警告
cmake_policy(SET CMP0074 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 设置
set(OpenCV_DIR "D:/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# PCL 设置
set(PCL_ROOT "D:/PCL/PCL 1.14.1")
set(PCL_DIR "${PCL_ROOT}/cmake")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加源文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 添加源文件
aux_source_directory(src SRC_FILES)
add_library(SensorData ${SRC_FILES})

# 添加可执行文件
add_executable(4DRadarTest main.cpp)

# 链接库文件
target_link_libraries(4DRadarTest 
    SensorData
    ${OpenCV_LIBS}    # 添加 OpenCV 库
    ${PCL_LIBRARIES}  # 添加 PCL 库
)


