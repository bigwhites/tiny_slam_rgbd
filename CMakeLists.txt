cmake_minimum_required(VERSION 3.5)
project(MY_SLAM)

# 指定C++标准
set( CMAKE_CXX_FLAGS "-std=c++23 -O3" )
set(CMAKE_BUILD_TYPE Release)


# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    "./include"
)

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/out/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/out/libs )

# 递归地查找src/main目录及其子目录下的所有.cpp文件
file(GLOB_RECURSE LIB_SOURCES "src/*.cpp")
file(GLOB_RECURSE MAIN_SOURCES "Main/*.cpp")



add_library(SLAM_LIB_S STATIC ${LIB_SOURCES})
add_library(SLAM_LIB SHARED ${LIB_SOURCES})
# 生成可执行文件
add_executable(slam_app ${MAIN_SOURCES})
# 链接库
target_link_libraries(SLAM_LIB
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    pthread  # 用于多线程
)
target_link_libraries(SLAM_LIB_S
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    pthread  # 用于多线程
)

target_link_libraries(slam_app SLAM_LIB)
