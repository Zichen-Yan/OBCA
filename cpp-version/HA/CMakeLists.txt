cmake_minimum_required(VERSION 3.10)
project(Hybrid_A_star)
set(TARGET_NAME HybridAstar_pathplanning_1.00.07)
set(CMAKE_BUILD_TYPE "Debug")

# Default to C17
#if(NOT CMAKE_C_STANDARD)
#  set(CMAKE_C_STANDARD 17)
#endif()


include_directories(${CMAKE_SOURCE_DIR}/include /usr/include/opencv4/opencv2/)
add_executable(${TARGET_NAME} main.cpp src/findpath.cpp src/dynamic_vertical.cpp src/dynamic_Level.cpp src/dynamic_Oblique.cpp src/dynamic_CC_road.cpp src/dynamic_multi.cpp)
find_package(OpenCV REQUIRED)
set(OpenCV_LIBS
    opencv_core
    opencv_video
    opencv_imgproc
    opencv_calib3d
    opencv_videoio
    opencv_highgui
    opencv_imgcodecs)
target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_link_libraries(${TARGET_NAME} ${EXTRA_LIBS} ${OpenCV_LIBS})
install(
  TARGETS ${TARGET_NAME} EXPORT ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


