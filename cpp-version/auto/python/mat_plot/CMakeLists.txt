cmake_minimum_required(VERSION 3.10)
# define the project name
set(TARGET_NAME "planplot")

# set the project name
project(${TARGET_NAME})
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# set(CMAKE_CXX_FLAGS "-Wall -fopenmp")
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(include)
add_executable(${TARGET_NAME} main.cpp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/byd2004/soft_test/mat_plot/)

target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_link_libraries(${TARGET_NAME} ${EXTRA_LIBS} ${PYTHON_LIBRARIES})
