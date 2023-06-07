# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(demo_pp_yolov5)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

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

set( SMS 30 32 35 37 50 52 53 60 61 62 70 72 75 87)
foreach(sm ${SMS})
	set(GENCODE ${GENCODE} -gencode arch=compute_${sm},code=sm_${sm})
endforeach()
set(HIGHEST_SM 87)
set(GENCODE ${GENCODE} -gencode arch=compute_${HIGHEST_SM},code=compute_${HIGHEST_SM})

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}
    -ccbin ${CMAKE_CXX_COMPILER}
    -Xcompiler -DWIN_INTERFACE_CUSTOM
    -Xcompiler -I/usr/aarch64-linux-gnu/include/
    -Xlinker -lsocket
    -Xlinker -rpath=/usr/lib/aarch64-linux-gnu/
    -Xlinker -rpath=/usr/aarch64-linux-gnu/lib/
    -Xlinker -L/usr/lib/aarch64-linux-gnu/
    -Xlinker -L/usr/aarch64-linux-gnu/lib/
)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  message("Using Debug Mode")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -g -G --ptxas-options=-v)
endif()

set(TENSORRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu/)
set(TENSORRT_LIBRARY_DIRS /usr/lib/aarch64-linux-gnu/)

include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIRS}
    include
    ${PROJECT_SOURCE_DIR}/include
    /usr/local/cuda/include
    /usr/lib/
    /usr/include/
)

link_directories(
	${TENSORRT_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	/usr/lib/aarch64-linux-gnu
  /usr/aarch64-linux-gnu/lib/
  /usr/include/
  /usr/local/cuda/lib64
  /usr/local/lib
  /usr/lib
)

file(GLOB_RECURSE SOURCE_FILES
    src/*.cu
    src/*.cpp
)

cuda_add_executable(${PROJECT_NAME} main_no_rviz_test.cpp ${SOURCE_FILES})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs visualization_msgs message_filters geometry_msgs custom_interfaces cv_bridge OpenCV PCL CUDA)

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/src/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

target_link_libraries(${PROJECT_NAME}
    libnvinfer.so
    libnvonnxparser.so
    libjsoncpp.a
)

target_link_libraries(demo_pp_yolov5 nvinfer)
target_link_libraries(demo_pp_yolov5 cudart)
target_link_libraries(demo_pp_yolov5 myplugins)
target_link_libraries(demo_pp_yolov5 ${OpenCV_LIBS})
target_link_libraries(demo_pp_yolov5 ${PCL_LIBRARIES})

install(TARGETS
	${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS myplugins
   DESTINATION lib/${PROJECT_NAME})
   
install(FILES launch/pp_yolov5_infer_launch_single.py
  DESTINATION share/${PROJECT_NAME}/launch)
if(UNIX)
add_definitions(-O2 -pthread)
endif(UNIX)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
