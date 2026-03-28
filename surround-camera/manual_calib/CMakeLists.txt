cmake_minimum_required(VERSION 2.8.3)
project(lidar2camera)
set(CMAKE_CXX_FLAGS "-std=c++14 -g -Wall -DHAVE_GLEW")

set(Pangolin_DIR /mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/Pangolin/build)
find_package(Pangolin REQUIRED)

# Manually add Pangolin include directories
set(PANGOLIN_ROOT /mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/Pangolin)
include_directories(${PANGOLIN_ROOT}/components/pango_core/include)
include_directories(${PANGOLIN_ROOT}/components/pango_display/include)
include_directories(${PANGOLIN_ROOT}/components/pango_geometry/include)
include_directories(${PANGOLIN_ROOT}/components/pango_glgeometry/include)
include_directories(${PANGOLIN_ROOT}/components/pango_image/include)
include_directories(${PANGOLIN_ROOT}/components/pango_opengl/include)
include_directories(${PANGOLIN_ROOT}/components/pango_packetstream/include)
include_directories(${PANGOLIN_ROOT}/components/pango_plot/include)
include_directories(${PANGOLIN_ROOT}/components/pango_scene/include)
include_directories(${PANGOLIN_ROOT}/components/pango_tools/include)
include_directories(${PANGOLIN_ROOT}/components/pango_vars/include)
include_directories(${PANGOLIN_ROOT}/components/pango_video/include)
include_directories(${PANGOLIN_ROOT}/components/pango_windowing/include)
include_directories(${PANGOLIN_ROOT}/components/tinyobj/include)
include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/SensorsCalibration/factory_calib/3rdparty/eigen3)
include_directories(/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/jsoncpp/include)
include_directories(/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/jsoncpp/include_wrapper)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} /mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/jsoncpp/build/lib/libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

add_executable(run_avm src/run_avm.cpp)
target_link_libraries(run_avm ${PROJECT_NAME})
target_link_libraries(run_avm ${Pangolin_LIBRARIES})
