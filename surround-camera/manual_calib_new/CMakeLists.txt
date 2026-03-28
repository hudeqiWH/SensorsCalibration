cmake_minimum_required(VERSION 3.10)
project(avm_manual_calib)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-g -Wall -O3 -DHAVE_GLEW")

# Pangolin
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

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Boost
find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIRS})

# Eigen
include_directories(/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/SensorsCalibration/factory_calib/3rdparty/eigen3)

# JsonCpp
include_directories(/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/jsoncpp/include)
include_directories(/mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/jsoncpp/include_wrapper)

# Source
include_directories(${PROJECT_SOURCE_DIR}/src)

# Output directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Executable
add_executable(run_avm_new src/run_avm_new.cpp)
target_link_libraries(run_avm_new 
    ${OpenCV_LIBS} 
    ${Pangolin_LIBRARIES} 
    ${Boost_SYSTEM_LIBRARY}
    /mnt/d327cf14-2b9b-4977-955d-e19c9242ea1a/deqi/r_calib/third_party/jsoncpp/build/lib/libjsoncpp.a
)
