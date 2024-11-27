cmake_minimum_required(VERSION 3.10)

project(4DRadarTest)

# 添加这一行来解决警告
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0144 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 取消Windows宏定义，防止PCL报错（重要）
if(WIN32)
    add_definitions(-DNOMINMAX)
    add_definitions(-D_SCL_SECURE_NO_WARNINGS)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

# 查找 OpenGL 包
find_package(OpenGL REQUIRED)
# 输出 OpenGL 版本
message(STATUS "OpenGL_INCLUDE_DIR: ${OPENGL_INCLUDE_DIR}")
message(STATUS "OpenGL_LIBRARIES: ${OPENGL_LIBRARIES}")
# 检查 OpenGL 版本（基于您找到的 OpenGL 版本）
if(OPENGL_VERSION)
    message(STATUS "OpenGL Version: ${OPENGL_VERSION}")
else()
    message(STATUS "OpenGL version info not available")
endif()


# 设置 FreeGLUT 的路径
set(FREEGLUT_INCLUDE_DIR "D:/freeglut/include")
set(FREEGLUT_LIB_DIR "D:/freeglut/lib/x64")
# 添加头文件和库路径
include_directories(${FREEGLUT_INCLUDE_DIR})
link_directories(${FREEGLUT_LIB_DIR})


# 设置 GLEW 路径
set(GLEW_INCLUDE_DIR "D:/glew-2.2.0/include")
set(GLEW_LIB_DIR "D:/glew-2.2.0/lib/Release/x64")
include_directories(${GLEW_INCLUDE_DIR})
link_directories(${GLEW_LIB_DIR})


# 设置PCL根目录（根据您的实际安装路径修改）
set(PCL_ROOT "D:/PCL/PCL 1.13.1")

# 添加PCL的cmake模块路径
list(APPEND CMAKE_PREFIX_PATH "${PCL_ROOT}/cmake")

# 添加其他必要的路径
set(PCL_INCLUDE_DIRS "${PCL_ROOT}/include/pcl-1.13")
set(PCL_LIBRARY_DIRS "${PCL_ROOT}/lib")
set(PCL_BINARY_DIRS "${PCL_ROOT}/bin")

# 查找 PCL 包
find_package(PCL 1.13 REQUIRED COMPONENTS common io visualization filters segmentation)

# 查找 VTK 包
find_package(VTK REQUIRED)
if (NOT VTK_FOUND)
    message(FATAL_ERROR "VTK not found")
else()
    message(STATUS "VTK found: ${VTK_VERSION}")
endif()

# OpenCV 设置
set(OpenCV_DIR "D:/opencv481/opencv/build/x64/vc16/lib")
find_package(OpenCV REQUIRED)


include_directories(
    ${OpenCV_INCLUDE_DIRS}
    "${PCL_ROOT}/3rdParty/Eigen/eigen3"
    "${PCL_ROOT}/3rdParty/VTK/include/vtk-9.2"
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIR}
)

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
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}    # 添加 OpenCV 库
    ${VTK_LIBRARIES}  # 链接 VTK 库
    freeglut opengl32 glu32
    glew32 OpenGL::GL
)

