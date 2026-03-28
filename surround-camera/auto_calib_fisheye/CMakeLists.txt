cmake_minimum_required(VERSION 2.8)
project(online_Calibration)
set(CMAKE_CXX_FLAGS "-std=c++17 -g -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
SET(CMAKE_BUILD_TYPE "Release")

# Find OpenMP for parallelization
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OPENMP)
    message(STATUS "OpenMP found, parallelization enabled")
else()
    message(STATUS "OpenMP not found, using sequential execution")
endif()

include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED )

# Find jsoncpp using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${JSONCPP_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib)
link_directories(${JSONCPP_LIBRARY_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${JSONCPP_LIBRARIES})

add_executable(run_AVM_Calibration src/calibration.cpp)
target_link_libraries(run_AVM_Calibration ${PROJECT_NAME} ${JSONCPP_LIBRARIES})

# BEV测试程序
add_executable(test_bev_simple test_bev_simple.cpp)
target_link_libraries(test_bev_simple ${PROJECT_NAME} ${JSONCPP_LIBRARIES})

# BEV无tail测试程序
add_executable(test_bev_no_tail test_bev_no_tail.cpp)
target_link_libraries(test_bev_no_tail ${PROJECT_NAME} ${JSONCPP_LIBRARIES})

# BEV单位矩阵测试程序
add_executable(test_bev_identity test_bev_identity.cpp)
target_link_libraries(test_bev_identity ${PROJECT_NAME} ${JSONCPP_LIBRARIES})

