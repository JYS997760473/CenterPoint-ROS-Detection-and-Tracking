cmake_minimum_required(VERSION 3.0.2)
project(centerpoint_detection)
set(arch ${CMAKE_HOST_SYSTEM_PROCESSOR})

find_package(CUDA REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  pcl_ros
  pcl_conversions
  tf
)

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL  "aarch64")
  set(CMAKE_C_COMPILER /usr/bin/aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/aarch64-linux-gnu-g++)
  set(CUDA_INSTALL_TARGET_DIR targets/aarch64-linux)
elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "x86_64")
  set(CMAKE_C_COMPILER /usr/bin/gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/g++)
  set(CUDA_INSTALL_TARGET_DIR targets/x86_64-linux)
endif()

set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)
set(CUDA_INCLUDE_DIRS ${CUDA_TOOLKIT_ROOT_DIR}/${CUDA_INSTALL_TARGET_DIR}/include)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-Wno-deprecated-declarations -O2")
add_compile_options(-W)
add_compile_options(-std=c++14)

set(TENSORRT_INCLUDE_DIRS /shared_data/TensorRT-8.6.1.6/include)
set(TENSORRT_LIBRARY_DIRS /shared_data/TensorRT-8.6.1.6/lib)

include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIRS}
  3rdparty/libspconv/include
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib pcl_ros pcl_conversions tf
  INCLUDE_DIRS include
  LIBRARIES pointpillars
)

link_directories(
  ${TENSORRT_LIBRARY_DIRS}
  3rdparty/libspconv/lib/${arch}
)

set(CenterPoint_files
    src/centerpoint.cpp
    src/postprocess.cpp
    src/postprocess_kernels.cu
    src/preprocess_kernels.cu
    src/preprocess.cpp
    src/tensorrt.cpp
)

cuda_add_library(centerpoint ${CenterPoint_files})
target_link_libraries(centerpoint
    libnvinfer_plugin.so.8
    libnvinfer.so
    libnvonnxparser.so
    libspconv.so
)

############################################################
# PCL
############################################################
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(
  ${PCL_LIBRARY_DIRS}
)

#--------- pointpillars_detection_lib --------------
add_library(centerpoint_detection_lib
  src/centerpoint_detection.cpp
)
target_link_libraries(centerpoint_detection_lib
  centerpoint
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#--------- pointpillars_detection node executable -----------
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  centerpoint_detection_lib
  ${PCL_LIBRARIES} 
  ${LINK_LIBRARY}
)
