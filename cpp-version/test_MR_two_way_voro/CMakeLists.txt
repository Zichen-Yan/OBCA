cmake_minimum_required(VERSION 3.10)
project(Hybrid_A_star)
set(TARGET_NAME HybridAstar_pathplanning_1.00.07)
set(CMAKE_BUILD_TYPE "Debug")


include_directories(${CMAKE_SOURCE_DIR}/include /usr/include/opencv4/opencv2/)
add_executable(${TARGET_NAME} main.cpp src/findpath.cpp src/dynamic_vertical.cpp src/dynamic_Level.cpp src/dynamic_Oblique.cpp src/dynamic_CC_road.cpp src/dynamic_multi.cpp
src/bucket_queue.cpp src/vec2d.cpp src/dynamic_voronoi.cpp)

find_package(OpenCV REQUIRED)
set(OpenCV_LIBS
    opencv_core
    opencv_video
    opencv_imgproc
    opencv_calib3d
    opencv_videoio
    opencv_highgui
    opencv_imgcodecs)
target_link_libraries(${TARGET_NAME} ${OpenCV_LIBS})


