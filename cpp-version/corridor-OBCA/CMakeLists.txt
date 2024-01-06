cmake_minimum_required(VERSION 3.13)
project(Hybrid_A_star)
set(TARGET_NAME HybridAstar_pathplanning_1.00.07)
set(CMAKE_BUILD_TYPE "Debug")

include_directories(${CMAKE_SOURCE_DIR}/include) # 添加头文件路径

add_executable(${TARGET_NAME} 
              main.cpp src/findpath.cpp src/dynamic_vertical.cpp 
              src/dynamic_Level.cpp src/dynamic_Oblique.cpp src/dynamic_CC_road.cpp 
              src/dynamic_multi.cpp src/obcaSolver.cpp src/dynamic_voronoi.cpp
              src/vec2d.cpp src/bucket_queue.cpp src/corridor.cpp)

find_package(Eigen3 REQUIRED)
find_package(casadi REQUIRED)

target_link_libraries(${TARGET_NAME} Eigen3::Eigen casadi) # 指定目标（exe或者so文件）需要包含的库


