cmake_minimum_required(VERSION 3.10)
# define the project name
set(TARGET_NAME "planAlgorithm")

# set the project name
project(${TARGET_NAME})


SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/common/include)

find_package(PythonLibs REQUIRED)
include_directories(./3rd/release/usr/matplotlib-cpp/)
include_directories( . )

aux_source_directory(${PROJECT_SOURCE_DIR}/common SRC_LIST1)
aux_source_directory(${PROJECT_SOURCE_DIR}/geometry SRC_LIST2)
aux_source_directory(${PROJECT_SOURCE_DIR}/hybridastar SRC_LIST3)
aux_source_directory(${PROJECT_SOURCE_DIR}/map SRC_LIST4)

# add executable
add_executable(${TARGET_NAME} main.cpp plot_result.cpp ${SRC_LIST1} ${SRC_LIST2} ${SRC_LIST3} ${SRC_LIST4})

#  target_link_directories(${NAME} PUBLIC ${IMPORTED_LIBS_PATH}) # method 2
target_compile_features(${TARGET_NAME} PUBLIC cxx_std_17)
target_link_libraries(${TARGET_NAME} PUBLIC ${EXTRA_LIBS} ${PYTHON_LIBRARIES})
target_include_directories(${TARGET_NAME} PUBLIC ${PYTHON_INCLUDE_DIRS})

install(
  TARGETS ${TARGET_NAME} EXPORT ${TARGET_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
