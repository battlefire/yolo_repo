# /**
# MIT License

# Copyright (c) 2018 NVIDIA CORPORATION. All rights reserved.

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# *
# */

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(trt-yolo-app LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wunused-function -Wunused-variable -Wfatal-errors")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(TRT_SDK_ROOT "" CACHE PATH "NVIDIA TensorRT SDK root path")
set(ARGUS_SDK_ROOT "" CACHE PATH "NVIDIA ARGUS SDK root path")
set(MULTI_MEDIA_ROOT "" CACHE PATH "NVIDIA MULTI MEDIA root path")
set(OPENCV_ROOT "" CACHE PATH "OpenCV SDK root path")

find_package(CUDA 10.0 EXACT REQUIRED cudart cublas curand)
list(APPEND GPU_ARCHS 30 35 37 50 52 60 61 70 75)


# Generate SASS for each architecture
foreach(arch ${GPU_ARCHS})
  set(GENCODES "${GENCODES} -gencode arch=compute_${arch},code=sm_${arch}")
endforeach()

# Generate PTX for the last architecture
list(GET GPU_ARCHS -1 LATEST_GPU_ARCH)
set(GENCODES "${GENCODES} -gencode arch=compute_${LATEST_GPU_ARCH},code=compute_${LATEST_GPU_ARCH}")

# Find OpenCV 
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui dnn video PATHS ${OPENCV_ROOT} ${CMAKE_SYSTEM_PREFIX_PATH} PATH_SUFFIXES build share NO_DEFAULT_PATH)
find_package(OpenCV REQUIRED core imgproc imgcodecs highgui dnn video)

# Find TensorRT
find_path(TRT_INCLUDE_DIR NvInfer.h HINTS ${TRT_SDK_ROOT} PATH_SUFFIXES include)
if(${TRT_INCLUDE_DIR} MATCHES "TRT_INCLUDE_DIR-NOTFOUND")
  MESSAGE(FATAL_ERROR "-- Unable to find TensorRT headers. Please set path using -DTRT_SDK_ROOT")
else()
  MESSAGE(STATUS "Found TensorRT headers at ${TRT_INCLUDE_DIR}")
endif()

find_library(TRT_LIBRARY_INFER nvinfer HINTS ${TRT_SDK_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
find_library(TRT_LIBRARY_INFER_PLUGIN nvinfer_plugin HINTS ${TRT_SDK_ROOT} PATH_SUFFIXES lib lib64 lib/x64)
if((${TRT_LIBRARY_INFER} MATCHES "TRT_LIBRARY_INFER-NOTFOUND") OR (${TRT_LIBRARY_INFER_PLUGIN} MATCHES "TRT_LIBRARY_INFER_PLUGIN-NOTFOUND"))
  MESSAGE(FATAL_ERROR "-- Unable to find TensorRT libs. Please set path using -DTRT_SDK_ROOT")
else()
  set(TRT_LIBRARY ${TRT_LIBRARY_INFER} ${TRT_LIBRARY_INFER_PLUGIN})
  MESSAGE(STATUS "Found TensorRT libs at ${TRT_LIBRARY}")
endif()

# Find argus
find_path(EGL_INCLUDE_DIR NvEglRenderer.h HINTS /usr/src/nvidia/tegra_multimedia_api/include/)
if(${EGL_INCLUDE_DIR} MATCHES "EGL_INCLUDE_DIR-NOTFOUND")
  MESSAGE(FATAL_ERROR "-- Unable to find EGL headers. Please set path using -DTRT_SDK_ROOT")
else()
  MESSAGE(STATUS "Found EGL headers at ${EGL_INCLUDE_DIR}")
endif()

find_library(ARGUS_LIBRARY_PRODUCER nveglstream_camconsumer HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES tegra)
find_library(ARGUS_LIBRARY_ARGUS nvargus_socketclient HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES tegra)
if((${ARGUS_LIBRARY_PRODUCER} MATCHES "ARGUS_LIBRARY_PRODUCER-NOTFOUND") OR (${ARGUS_LIBRARY_ARGUS} MATCHES "ARGUS_LIBRARY_ARGUS-NOTFOUND"))
  MESSAGE(FATAL_ERROR "-- Unable to find EGL libs. Please set path using -DTRT_SDK_ROOT")
else()
  set(EGL_LIBRARY ${ARGUS_LIBRARY_PRODUCER} ${ARGUS_LIBRARY_ARGUS})
  MESSAGE(STATUS "Found EGL libs at ${EGL_LIBRARY}")
endif()

# Find X11
#find_path(X11_INCLUDE_DIR gl2.h HINTS /usr/include/X11)
find_library(X11_LIBRARY X11 HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES lib)
MESSAGE(STATUS "Found X11 libs at ${X11_LIBRARY}")
#find_path(X11_XMU_INCLUDE_DIR Error.h HINTS /usr/include/X11/Xmu)

#Find pthead
find_path(PTHREAD_INCLUDE_DIR Thread.h HINTS /usr/src/nvidia/tegra_multimedia_api/argus/samples/utils)

#Find JPEG
find_path(JPEG_INCLUDE_DIR jpeglib.h HINTS /usr/src/nvidia/tegra_multimedia_api/include/libjpeg-8b)
find_library(JPEG_LIBRARY nvjpeg HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES tegra)
if(${JPEG_LIBRARY} MATCHES "JPEG_LIBRARY-NOTFOUND")
  MESSAGE(FATAL_ERROR "-- Unable to find JPEG_LIBRARY.")
else()
  MESSAGE(STATUS "Found nvjpeg libs at ${JPEG_LIBRARY}")
endif()

#Find GLESv2
find_path(GLESv2_INCLUDE_DIR gl2.h HINTS /usr/include/GLES2)
find_library(GLESv2_LIBRARY GLESv2 HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES lib)

#Find EGL headers
find_path(EGL_INCLUDE egl.h HINTS /usr/include/EGL)
find_library(EGL_HEADER_LIBRARY EGL HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES lib)

#Find nvbuf_utils
find_library(NVBUF_UTILS_LIBRARY nvbuf_utils HINTS ${ARGUS_SDK_ROOT} PATH_SUFFIXES tegra)
if(${NVBUF_UTILS_LIBRARY} MATCHES "NVBUF_UTILS_LIBRARY-NOTFOUND")
  MESSAGE(FATAL_ERROR "-- Unable to find NVBUF_UTILS_LIBRARY.")
else()
  MESSAGE(STATUS "Found nvbuf_utils libs at ${NVBUF_UTILS_LIBRARY}")
endif()

#Find JSON
find_path(JSON_C_INCLUDE_DIR json.h HINTS ${CMAKE_SOURCE_DIR}/../../../json-c/)
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so.4")
find_library(JSON_C_LIBRARY json-c HINTS ${CMAKE_SOURCE_DIR}/../../../json-c/)
if(${JSON_C_LIBRARY} MATCHES "JSON_C_LIBRARY-NOTFOUND")
  MESSAGE(FATAL_ERROR "-- Unable to find JSON_C_LIBRARY.")
else()
  MESSAGE(STATUS "Found json libs at ${JSON_C_LIBRARY}")
endif()

# Add yolo lib as subdir
add_subdirectory(${PROJECT_SOURCE_DIR}/../../lib ${PROJECT_BINARY_DIR}/lib)

include_directories(${CUDA_INCLUDE_DIRS} ${TRT_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/../../lib ${EGL_INCLUDE_DIR} ${GLESv2_INCLUDE_DIR} ${EGL_INCLUDE} ${X11_XMU_INCLUDE_DIR} ${PTHREAD_INCLUDE_DIR} ${JPEG_INCLUDE_DIR} ${JSON_C_INCLUDE_DIR})
link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)

add_executable(trt-yolo-app trt-yolo-app.cpp NvEglRenderer.cpp NvLogging.cpp NvElement.cpp NvElementProfiler.cpp NvJpegEncoder.cpp Thread.cpp uartwireless.cpp uart.cpp LaneDetector.cpp TcpClient_4g.cpp TransferJson.cpp)
target_link_libraries(trt-yolo-app yolo-lib)

#Create directory to save detections
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/../../data/detections)

#Install app
install(TARGETS trt-yolo-app RUNTIME DESTINATION bin CONFIGURATIONS Release Debug)
