cmake_minimum_required(VERSION 3.16)

set(SENTINEL_TARGET_PLATFORM "thor" CACHE STRING "Target platform: thor or x86")
set_property(CACHE SENTINEL_TARGET_PLATFORM PROPERTY STRINGS thor x86)
if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    set(BUILDTOOLS_TARGET_PLATFORM "thor")
elseif(SENTINEL_TARGET_PLATFORM STREQUAL "x86")
    # Leave BUILDTOOLS_TARGET_PLATFORM unset for host validation builds. Some
    # ADBuildTools packages reject a literal "x86" value but configure host
    # builds correctly when the platform is not preselected.
else()
    message(FATAL_ERROR "Unsupported SENTINEL_TARGET_PLATFORM=${SENTINEL_TARGET_PLATFORM}; use thor or x86.")
endif()

set(SENTINEL_THOR_ROOT "" CACHE PATH "Thor SDK root containing aarch64 toolchain and CUDA")
if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    set(SENTINEL_THOR_ROOT_CANDIDATES)
    if(SENTINEL_THOR_ROOT)
        list(APPEND SENTINEL_THOR_ROOT_CANDIDATES ${SENTINEL_THOR_ROOT})
    endif()
    list(APPEND SENTINEL_THOR_ROOT_CANDIDATES
        /home/wslab/thor_test_container/usr/local/thor
        /tmp/t
        /usr/local/thor)
    foreach(_thor_root ${SENTINEL_THOR_ROOT_CANDIDATES})
        if(EXISTS ${_thor_root}/aarch64--glibc--bleeding-edge-2024.02-1/bin/aarch64-buildroot-linux-gnu-g++
           AND EXISTS ${_thor_root}/cuda-12.8/targets/aarch64-linux/include/cuda_runtime.h)
            set(SENTINEL_THOR_ROOT ${_thor_root})
            break()
        endif()
    endforeach()
    if(NOT SENTINEL_THOR_ROOT)
        message(FATAL_ERROR "Thor toolchain not found. Set -DSENTINEL_THOR_ROOT=/path/to/usr/local/thor.")
    endif()
    message(STATUS "Using Thor root: ${SENTINEL_THOR_ROOT}")

    set(SENTINEL_THOR_GLIBC_DIR
        ${SENTINEL_THOR_ROOT}/aarch64--glibc--bleeding-edge-2024.02-1)
    set(SENTINEL_THOR_SYSROOT
        ${SENTINEL_THOR_GLIBC_DIR}/aarch64-buildroot-linux-gnu/sysroot)
    if(EXISTS ${SENTINEL_THOR_GLIBC_DIR}/bin/aarch64-buildroot-linux-gnu-gcc)
        set(CMAKE_C_COMPILER
            ${SENTINEL_THOR_GLIBC_DIR}/bin/aarch64-buildroot-linux-gnu-gcc
            CACHE FILEPATH "Thor C compiler" FORCE)
    endif()
    if(EXISTS ${SENTINEL_THOR_GLIBC_DIR}/bin/aarch64-buildroot-linux-gnu-g++)
        set(CMAKE_CXX_COMPILER
            ${SENTINEL_THOR_GLIBC_DIR}/bin/aarch64-buildroot-linux-gnu-g++
            CACHE FILEPATH "Thor CXX compiler" FORCE)
    endif()
endif()

project(sentinel_perception)

option(SENTINEL_DIAGNOSTIC_NO_L3
       "Build bag-replay diagnostics without unavailable vehicle l3_msgs headers/publishers"
       OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(SENSEAUTO_ROOT "/opt/senseauto" CACHE PATH "SenseAuto package root")
set(SENSEAUTO_ACTIVE_ROOT "/opt/senseauto_active" CACHE PATH "SenseAuto active package root")
set(SENTINEL_SENSEAUTO_DEV_ROOT "" CACHE PATH "SenseAuto installed root used by ADBuildTools")
option(SENTINEL_ENABLE_FFMPEG_DECODER "Build FFmpeg/NVDEC HEVC decoder backend" ON)
option(SENTINEL_USE_SYSTEM_FFMPEG "Use system FFmpeg instead of SenseAuto bundled FFmpeg" OFF)

set(SENTINEL_TOOL_SHIM_DIR ${CMAKE_BINARY_DIR}/cmake-bin)
file(MAKE_DIRECTORY ${SENTINEL_TOOL_SHIM_DIR})
find_program(CCACHE_PROGRAM ccache)
if(NOT CCACHE_PROGRAM)
    file(WRITE ${SENTINEL_TOOL_SHIM_DIR}/ccache
        "#!/bin/sh\n"
        "case \"$1\" in\n"
        "  /usr/local/thor/*)\n"
        "    _compiler=\"${SENTINEL_THOR_ROOT}/\${1#/usr/local/thor/}\"\n"
        "    shift\n"
        "    exec \"$_compiler\" \"$@\"\n"
        "    ;;\n"
        "esac\n"
        "exec \"$@\"\n")
    execute_process(COMMAND chmod +x ${SENTINEL_TOOL_SHIM_DIR}/ccache)
    set(CMAKE_PROGRAM_PATH ${SENTINEL_TOOL_SHIM_DIR} ${CMAKE_PROGRAM_PATH})
    set(ENV{PATH} "${SENTINEL_TOOL_SHIM_DIR}:$ENV{PATH}")
endif()
if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    # Some SenseAuto SDK images require `ccache` from ADBuildTools even when the
    # host/container does not install it. Provide a no-op launcher so configuration
    # can proceed without changing the container image.
    file(WRITE ${SENTINEL_TOOL_SHIM_DIR}/link-launcher
        "#!/bin/sh\n"
        "_args=\n"
        "for _arg in \"$@\"; do\n"
        "  case \"$_arg\" in\n"
        "    -lz)\n"
        "      _arg=\"${CMAKE_BINARY_DIR}/compat-libs/libz.so\"\n"
        "      ;;\n"
        "    *'/usr/local/thor/'*)\n"
        "      _arg=$(printf '%s' \"$_arg\" | sed 's#/usr/local/thor/#${SENTINEL_THOR_ROOT}/#g')\n"
        "      ;;\n"
        "  esac\n"
        "  _args=\"$_args '$(printf '%s' \"$_arg\" | sed \"s/'/'\\\\\\\\''/g\")'\"\n"
        "done\n"
        "eval \"set -- $_args\"\n"
        "exec \"$@\"\n")
    execute_process(COMMAND chmod +x ${SENTINEL_TOOL_SHIM_DIR}/link-launcher)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ${SENTINEL_TOOL_SHIM_DIR}/link-launcher)
endif()

# ====================== SenseAuto SDK ======================
find_package(ADBuildTools REQUIRED PATHS ${SENSEAUTO_ACTIVE_ROOT}/senseauto-buildtools)
include(${BUILDTOOLS_UTILS_CMAKE_DIR}/SenseIdlUtils.cmake)
if(SENTINEL_SENSEAUTO_DEV_ROOT)
    set(SENSEAUTO_DEV_INSTALLED_ROOT "${SENTINEL_SENSEAUTO_DEV_ROOT}")
    set(SENSEAUTO_DEV_INSTALLED_ROOT "${SENTINEL_SENSEAUTO_DEV_ROOT}" CACHE PATH "SenseAuto installed root used by ADBuildTools" FORCE)
endif()

include_directories(${SENSEAUTO_ACTIVE_ROOT}/senseauto-pal/include)
link_directories(${SENSEAUTO_ACTIVE_ROOT}/senseauto-pal/lib)

include_directories(${SENSEAUTO_ACTIVE_ROOT}/senseauto-swcfl/include)
link_directories(${SENSEAUTO_ACTIVE_ROOT}/senseauto-swcfl/lib)

include_directories(${SENSEAUTO_ACTIVE_ROOT}/senseauto-framework-sdk/include)
link_directories(${SENSEAUTO_ACTIVE_ROOT}/senseauto-framework-sdk/lib)

find_senseauto_package(senseauto-framework-sdk ADFramework REQUIRED
    COMPONENTS ad_log ad_common ad_scm ad_time ad_coordutils ad_utils ad_rscl ad_msg_bridge)
find_senseauto_package(senseauto-msgs ADMsgIDL REQUIRED)

if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    # ADBuildTools may inject an absolute /usr/local/thor linker even when the
    # compiler cache points to the detected Thor SDK. Keep the generated link step
    # tied to the compiler selected above so laptop builds without /usr/local/thor
    # symlinks work.
    set(CMAKE_CXX_LINK_EXECUTABLE
        "${SENTINEL_TOOL_SHIM_DIR}/link-launcher ${CMAKE_CXX_COMPILER} <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
endif()

# ====================== CUDA / TensorRT ======================
set(CUDA_LIBS)
set(TENSORRT_LIBS)
set(THOR_DRIVE_LIB_DIR)
if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    set(CUDA_TOOLKIT_ROOT_DIR "${SENTINEL_THOR_ROOT}/cuda-12.8" CACHE PATH "CUDA Toolkit for Thor")
    set(CUDA_INCLUDE_DIRS "${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/include")
    set(CUDA_LIB_DIR "${CUDA_TOOLKIT_ROOT_DIR}/targets/aarch64-linux/lib")
    set(CUDA_FOUND TRUE)
    set(CUDA_VERSION "12.8")
    set(CUDA_CUDART_LIBRARY "${CUDA_LIB_DIR}/libcudart.so")
    include_directories(${CUDA_INCLUDE_DIRS})
    link_directories(${CUDA_LIB_DIR})
    set(CUDA_LIBS cudart)

    set(TENSORRT_ROOT "${SENTINEL_THOR_ROOT}/aarch64-linux-gnu")
    set(TENSORRT_INCLUDE_DIR "${TENSORRT_ROOT}/include")
    set(TENSORRT_LIB_DIR "${TENSORRT_ROOT}/lib")
    include_directories(${TENSORRT_INCLUDE_DIR})
    link_directories(${TENSORRT_LIB_DIR})
    add_definitions(-DTRT_10)
    set(TENSORRT_LIBS nvinfer nvinfer_plugin nvonnxparser)

    set(THOR_DRIVE_LIB_DIR "${SENTINEL_THOR_ROOT}/drive-linux/lib")
    link_directories(${THOR_DRIVE_LIB_DIR})
else()
    find_package(CUDA QUIET)
    if(CUDA_FOUND)
        include_directories(${CUDA_INCLUDE_DIRS})
        link_directories(${CUDA_TOOLKIT_ROOT_DIR}/lib64)
        set(CUDA_LIBS cudart)
    endif()

    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        PATHS /usr/include/x86_64-linux-gnu /usr/include
              /usr/local/cuda/include /usr/local/TensorRT/include /opt/TensorRT/include)
    find_library(TENSORRT_NVINFER_LIB nvinfer
        PATHS /usr/lib/x86_64-linux-gnu /usr/lib
              /usr/local/cuda/lib64 /usr/local/TensorRT/lib /opt/TensorRT/lib)
    find_library(TENSORRT_NVINFER_PLUGIN_LIB nvinfer_plugin
        PATHS /usr/lib/x86_64-linux-gnu /usr/lib
              /usr/local/cuda/lib64 /usr/local/TensorRT/lib /opt/TensorRT/lib)
    find_library(TENSORRT_NVONNXPARSER_LIB nvonnxparser
        PATHS /usr/lib/x86_64-linux-gnu /usr/lib
              /usr/local/cuda/lib64 /usr/local/TensorRT/lib /opt/TensorRT/lib)
    if(TENSORRT_INCLUDE_DIR)
        include_directories(${TENSORRT_INCLUDE_DIR})
    endif()
    foreach(_trt_lib ${TENSORRT_NVINFER_LIB} ${TENSORRT_NVINFER_PLUGIN_LIB} ${TENSORRT_NVONNXPARSER_LIB})
        if(_trt_lib)
            list(APPEND TENSORRT_LIBS ${_trt_lib})
        endif()
    endforeach()
endif()

# ====================== OpenCV (manual, to bypass CUDA version check) ======================
set(OpenCV_ROOT_DIR ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/opencv)
if(NOT EXISTS ${OpenCV_ROOT_DIR}/include/opencv4/opencv2/opencv.hpp
   AND EXISTS ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/opencv4/include/opencv4/opencv2/opencv.hpp)
    set(OpenCV_ROOT_DIR ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/opencv4)
endif()
set(OpenCV_INCLUDE_DIRS
    ${OpenCV_ROOT_DIR}/include
    ${OpenCV_ROOT_DIR}/include/opencv4)
set(OpenCV_LIB_DIR ${OpenCV_ROOT_DIR}/lib)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

set(OpenCV_LIBS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    opencv_calib3d
    opencv_video
    opencv_dnn
)

# ====================== FFmpeg (NVDEC) - ARM copy ======================
set(FFMPEG_LIBS)
set(FFMPEG_LIB_DIR)
if(SENTINEL_ENABLE_FFMPEG_DECODER)
    add_definitions(-DSENTINEL_ENABLE_FFMPEG_DECODER=1)
    if(SENTINEL_USE_SYSTEM_FFMPEG)
        set(FFMPEG_ROOT_DIR /usr)
    else()
        set(FFMPEG_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg-4.3.2-he2e520b_0)
    endif()
    if(FFMPEG_ROOT_DIR STREQUAL "/usr"
       AND (EXISTS /usr/include/libavcodec/avcodec.h
            OR EXISTS /usr/include/x86_64-linux-gnu/libavcodec/avcodec.h))
        set(FFMPEG_ROOT_DIR /usr)
    elseif(NOT EXISTS ${FFMPEG_ROOT_DIR}/include/libavcodec/avcodec.h)
        if(EXISTS ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/ffmpeg/include/libavcodec/avcodec.h)
            set(FFMPEG_ROOT_DIR ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/ffmpeg)
        elseif(EXISTS /usr/include/libavcodec/avcodec.h
               OR EXISTS /usr/include/x86_64-linux-gnu/libavcodec/avcodec.h)
            set(FFMPEG_ROOT_DIR /usr)
        else()
            message(FATAL_ERROR "FFmpeg not found in ${FFMPEG_ROOT_DIR} or ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/ffmpeg")
        endif()
    endif()

    if(FFMPEG_ROOT_DIR STREQUAL "/usr")
        if(EXISTS /usr/include/x86_64-linux-gnu/libavcodec/avcodec.h)
            set(FFMPEG_INCLUDE_DIR /usr/include/x86_64-linux-gnu)
        else()
            set(FFMPEG_INCLUDE_DIR /usr/include)
        endif()
        set(FFMPEG_LIB_DIR /usr/lib/x86_64-linux-gnu)
    else()
        set(FFMPEG_INCLUDE_DIR ${FFMPEG_ROOT_DIR}/include)
        set(FFMPEG_LIB_DIR ${FFMPEG_ROOT_DIR}/lib)
    endif()
    include_directories(${FFMPEG_INCLUDE_DIR})
    link_directories(${FFMPEG_LIB_DIR})
    find_library(FFMPEG_AVCODEC_LIB NAMES avcodec avcodec-ffmpeg PATHS ${FFMPEG_LIB_DIR} NO_DEFAULT_PATH)
    find_library(FFMPEG_AVUTIL_LIB NAMES avutil avutil-ffmpeg PATHS ${FFMPEG_LIB_DIR} NO_DEFAULT_PATH)
    find_library(FFMPEG_AVFORMAT_LIB NAMES avformat avformat-ffmpeg PATHS ${FFMPEG_LIB_DIR} NO_DEFAULT_PATH)
    find_library(FFMPEG_SWSCALE_LIB NAMES swscale swscale-ffmpeg PATHS ${FFMPEG_LIB_DIR} NO_DEFAULT_PATH)
    find_library(FFMPEG_AVFILTER_LIB NAMES avfilter avfilter-ffmpeg PATHS ${FFMPEG_LIB_DIR} NO_DEFAULT_PATH)
    set(FFMPEG_LIBS
        ${FFMPEG_AVCODEC_LIB}
        ${FFMPEG_AVUTIL_LIB}
        ${FFMPEG_AVFORMAT_LIB}
        ${FFMPEG_SWSCALE_LIB})
    if(FFMPEG_AVFILTER_LIB)
        list(APPEND FFMPEG_LIBS ${FFMPEG_AVFILTER_LIB})
    endif()
    message(STATUS "FFmpeg root: ${FFMPEG_ROOT_DIR}")
    message(STATUS "FFmpeg include: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS "FFmpeg lib dir: ${FFMPEG_LIB_DIR}")
    message(STATUS "FFmpeg libs: ${FFMPEG_LIBS}")
endif()

set(FFMPEG_FALLBACK_LIB_DIR ${SENTINEL_SENSEAUTO_DEV_ROOT}/opt/libs-xavier/lib)
if(NOT FFMPEG_LIBS AND EXISTS ${FFMPEG_FALLBACK_LIB_DIR}/libavcodec.so)
    link_directories(${FFMPEG_FALLBACK_LIB_DIR})
    set(FFMPEG_LIB_DIR ${FFMPEG_FALLBACK_LIB_DIR})
    set(FFMPEG_LIBS avcodec avutil avformat swscale)
endif()

# SenseAuto 3rdparty libs from the user-provided Thor CMake.
if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    set(SENSEAUTO_3RDPARTY_LIB ${SENSEAUTO_ROOT}/senseauto-3rdparty/0.251127.1344+/3rdparty/lib)
else()
    set(SENSEAUTO_3RDPARTY_LIB ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/lib)
endif()
if(EXISTS ${SENSEAUTO_3RDPARTY_LIB})
    link_directories(${SENSEAUTO_3RDPARTY_LIB})
endif()

set(EXTERNAL_ARM_LIB_DIRS
    /opt/libs-xavier/lib
)
foreach(_ext_dir ${EXTERNAL_ARM_LIB_DIRS})
    if(EXISTS ${_ext_dir})
        link_directories(${_ext_dir})
    endif()
endforeach()

# ====================== YAML-CPP ======================
find_package(yaml-cpp QUIET)

# ====================== Local Cap'n Proto: StateBroadcast ======================
set(CAPNPC_SRC_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/idl)
set(CAPNPC_OUTPUT_DIR ${CMAKE_BINARY_DIR}/idl_gen)
file(MAKE_DIRECTORY ${CAPNPC_OUTPUT_DIR})
capnp_generate_cpp_sense(STATE_BROADCAST_CAPNP_SRCS STATE_BROADCAST_CAPNP_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/idl/state_broadcast.capnp
)
add_library(state_broadcast_capnp STATIC ${STATE_BROADCAST_CAPNP_SRCS})
target_include_directories(state_broadcast_capnp PUBLIC
    ${CAPNPC_OUTPUT_DIR}
    ${PATH_SOURCE_3RDPARTY}/include
    ${PATH_SOURCE_3RDPARTY}/capnp/include
)
target_link_libraries(state_broadcast_capnp PUBLIC kj capnp)
set_property(TARGET state_broadcast_capnp PROPERTY CXX_STANDARD 14)
set_property(TARGET state_broadcast_capnp PROPERTY POSITION_INDEPENDENT_CODE ON)

# ====================== Common include ======================
include_directories(
    ${ADFramework_INCLUDE_DIRS}
    ${ADMsgIDL_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CAPNPC_OUTPUT_DIR}
    rscl_access
)

# ====================== UUID ======================
set(SENTINEL_COMPAT_LIB_DIR ${CMAKE_BINARY_DIR}/compat-libs)
file(MAKE_DIRECTORY ${SENTINEL_COMPAT_LIB_DIR})
set(UUID_LIB "")
find_library(UUID_LIB_SENSE NAMES uuid
    PATHS ${SENSEAUTO_3RDPARTY_LIB} ${SENSEAUTO_ACTIVE_ROOT}/senseauto-3rdparty/3rdparty/lib
    NO_DEFAULT_PATH)
if(UUID_LIB_SENSE)
    set(UUID_LIB ${UUID_LIB_SENSE})
else()
    find_library(UUID_LIB_SYSTEM NAMES uuid
        PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu /usr/lib /lib)
    if(UUID_LIB_SYSTEM)
        set(UUID_LIB ${UUID_LIB_SYSTEM})
    elseif(SENTINEL_TARGET_PLATFORM STREQUAL "x86")
        find_library(UUID_LIB_RUNTIME NAMES libuuid.so.1
            PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu /usr/lib /lib
            NO_DEFAULT_PATH)
        if(UUID_LIB_RUNTIME)
            execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
                ${UUID_LIB_RUNTIME}
                ${SENTINEL_COMPAT_LIB_DIR}/libuuid.so)
            set(UUID_LIB ${SENTINEL_COMPAT_LIB_DIR}/libuuid.so)
        else()
            set(UUID_LIB uuid)
        endif()
    else()
        set(UUID_LIB uuid)
    endif()
endif()

if(UUID_LIB)
    message(STATUS "UUID library: ${UUID_LIB}")
else()
    set(UUID_LIB uuid)
endif()

if(SENTINEL_TARGET_PLATFORM STREQUAL "thor"
   AND EXISTS ${SENTINEL_THOR_ROOT}/aarch64-linux-gnu/lib/libz.so.1
   AND NOT EXISTS ${SENTINEL_THOR_ROOT}/aarch64-linux-gnu/lib/libz.so)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${SENTINEL_THOR_ROOT}/aarch64-linux-gnu/lib/libz.so.1
        ${SENTINEL_COMPAT_LIB_DIR}/libz.so)
endif()
link_directories(${SENTINEL_COMPAT_LIB_DIR})

# The diagnostic replay build can run on hosts where vehicle l3_msgs are not
# installed. Production/vehicle builds keep the real L3 publisher and library.
set(SENTINEL_L3_LIBS l3_msgs_capnp)
if(SENTINEL_DIAGNOSTIC_NO_L3)
    set(SENTINEL_L3_LIBS "")
endif()

# ====================== Common libraries ======================
set(COMMON_LIBS
    ${ADFramework_LIBRARIES}
    ${OpenCV_LIBS}
    ad_localization_capnp
    ad_gac_capnp
    ad_sensor_capnp
    ad_vehicle_capnp
    avp_pdc_meb_capnp
    avp_uss_capnp
    ${SENTINEL_L3_LIBS}
    state_broadcast_capnp
    kj capnp
    ad_rscl_cuda
    ${CUDA_LIBS}
    ${TENSORRT_LIBS}
    -lstdc++fs
    yaml-cpp
    ${UUID_LIB}
)

# ====================== Sources ======================
set(RSCL_ACCESS_SRC rscl_access/rscl_access.cpp)
set(DECODER_SRC
    nvmedia_hevc_decoder.cpp
    hevc_decoder_factory.cpp
)
if(SENTINEL_ENABLE_FFMPEG_DECODER)
    list(APPEND DECODER_SRC nv_hevc_decoder.cpp)
endif()
set(IMU_SRC imu/imu_subscriber.cpp)
set(USS_SRC uss/uss_subscriber.cpp)
set(LIDAR_SRC lidar/lidar_subscriber.cpp)
set(CAMERA_SRC camera/camera_subscriber.cpp camera/fisheye_undistort.cpp)
set(DETECTION_SRC detection/detector.cpp detection/tracker.cpp)
set(USS_VISION_SRC fusion/uss_vision_fusion.cpp)
set(ALERT_SRC alert/alert_manager.cpp)
set(STATE_SRC state/sentry_state_machine.cpp)
set(ORSM_SRC
    orsm/orsm_state_machine.cpp
    orsm/single_orsm_manager.cpp
    orsm/multi_vehicle_orsm_manager.cpp
    orsm/orsm_manager.cpp
)
if(SENTINEL_DIAGNOSTIC_NO_L3)
    set(PUBLISH_SRC publish/rscl_publisher_diagnostic.cpp)
else()
    set(PUBLISH_SRC publish/rscl_publisher.cpp)
endif()
set(CALIB_SRC calib/sensor_calib_manager.cpp)
set(RECORDING_SRC recording/video_recorder.cpp)
set(NODE_SRC sentinel_node.cpp)

# ====================== Main target ======================
add_executable(sentinel_perception
    ${RSCL_ACCESS_SRC}
    ${DECODER_SRC}
    ${IMU_SRC}
    ${USS_SRC}
    ${LIDAR_SRC}
    ${CAMERA_SRC}
    ${DETECTION_SRC}
    ${USS_VISION_SRC}
    ${ALERT_SRC}
    ${STATE_SRC}
    ${ORSM_SRC}
    ${PUBLISH_SRC}
    ${CALIB_SRC}
    ${RECORDING_SRC}
    ${NODE_SRC}
    main.cpp
)

if(SENTINEL_DIAGNOSTIC_NO_L3)
    target_compile_definitions(sentinel_perception PRIVATE SENTINEL_DIAGNOSTIC_NO_L3=1)
endif()
if(CUDA_LIBS AND TENSORRT_LIBS)
    target_compile_definitions(sentinel_perception PRIVATE SENTINEL_HAS_TENSORRT=1)
endif()

target_link_directories(sentinel_perception BEFORE PRIVATE ${SENTINEL_COMPAT_LIB_DIR})
target_compile_options(sentinel_perception PRIVATE -Wno-deprecated-declarations)

target_link_libraries(sentinel_perception
    ${COMMON_LIBS}
    -ldl
)
if(SENTINEL_ENABLE_FFMPEG_DECODER)
    target_link_libraries(sentinel_perception
        "-Wl,--whole-archive"
        ${FFMPEG_LIBS}
        "-Wl,--no-whole-archive")
elseif(FFMPEG_LIBS)
    target_link_libraries(sentinel_perception ${FFMPEG_LIBS})
endif()

target_link_options(sentinel_perception PRIVATE
    "-L${SENTINEL_COMPAT_LIB_DIR}"
    "-Wl,--allow-shlib-undefined"
    "-Wl,--disable-new-dtags"
    "-Wl,-rpath-link,${SENSEAUTO_3RDPARTY_LIB}"
)
if(SENTINEL_TARGET_PLATFORM STREQUAL "thor")
    target_link_options(sentinel_perception PRIVATE
        "-Wl,-rpath-link,${THOR_DRIVE_LIB_DIR}"
        "-Wl,-rpath-link,${SENTINEL_THOR_SYSROOT}/usr/lib"
        "-Wl,-rpath-link,${SENTINEL_THOR_SYSROOT}/lib")
endif()
if(SENTINEL_ENABLE_FFMPEG_DECODER)
    target_link_options(sentinel_perception PRIVATE "-Wl,-rpath-link,${FFMPEG_LIB_DIR}")
elseif(FFMPEG_LIB_DIR)
    target_link_options(sentinel_perception PRIVATE "-Wl,-rpath-link,${FFMPEG_LIB_DIR}")
endif()
foreach(_ext_dir ${EXTERNAL_ARM_LIB_DIRS})
    if(EXISTS ${_ext_dir})
        target_link_options(sentinel_perception PRIVATE "-Wl,-rpath-link,${_ext_dir}")
    endif()
endforeach()

set_target_properties(sentinel_perception PROPERTIES
    BUILD_RPATH "$ORIGIN/lib;$ORIGIN"
    INSTALL_RPATH "$ORIGIN/lib;$ORIGIN"
)

# Host-side diagnostic tests run directly from the build directory rather than
# from a packaged runtime tree, so retain the SDK paths required by their
# transitive dependencies.
set(SENTINEL_TEST_RUNTIME_RPATH
    "${CMAKE_BINARY_DIR}/compat-libs;${FFMPEG_LIB_DIR};${OpenCV_ROOT_DIR}/lib;/opt/senseauto_active/senseauto-3rdparty/3rdparty/lib;/opt/senseauto_active/senseauto-msgs/lib;/opt/senseauto_active/senseauto-rscl/lib"
)

include(CTest)
if(BUILD_TESTING)
    add_executable(rfdetr_detector_smoke
        tools/rfdetr_detector_smoke.cpp
        detection/detector.cpp
    )
    target_link_libraries(rfdetr_detector_smoke
        ${OpenCV_LIBS}
        ${CUDA_LIBS}
        ${TENSORRT_LIBS}
    )
    set_target_properties(rfdetr_detector_smoke PROPERTIES
        BUILD_RPATH "${SENTINEL_TEST_RUNTIME_RPATH}"
    )

    add_executable(lidar_bev_video tools/lidar_bev_video.cpp)
    target_link_libraries(lidar_bev_video ${COMMON_LIBS} ${OpenCV_LIBS} opencv_videoio)
    target_link_options(lidar_bev_video PRIVATE "-Wl,--allow-shlib-undefined" "-Wl,--disable-new-dtags")
    set_target_properties(lidar_bev_video PROPERTIES BUILD_RPATH "${SENTINEL_TEST_RUNTIME_RPATH}")

    add_executable(orsm_response_feedback_test
        tests/orsm_response_feedback_test.cpp
        orsm/orsm_state_machine.cpp
        orsm/single_orsm_manager.cpp
        orsm/multi_vehicle_orsm_manager.cpp
    )
    target_include_directories(orsm_response_feedback_test SYSTEM PRIVATE
        /opt/senseauto_active/senseauto-3rdparty/3rdparty/include
    )
    set(ORSM_RESPONSE_TEST_LIBS ${COMMON_LIBS})
    list(REMOVE_ITEM ORSM_RESPONSE_TEST_LIBS l3_msgs_capnp)
    target_link_libraries(orsm_response_feedback_test ${ORSM_RESPONSE_TEST_LIBS})
    target_link_options(orsm_response_feedback_test PRIVATE
        "-Wl,--allow-shlib-undefined"
        "-Wl,--disable-new-dtags"
    )
    set_target_properties(orsm_response_feedback_test PROPERTIES
        BUILD_RPATH "${SENTINEL_TEST_RUNTIME_RPATH}"
    )
    add_test(NAME orsm_response_feedback_test COMMAND orsm_response_feedback_test)
    set_tests_properties(orsm_response_feedback_test PROPERTIES
        ENVIRONMENT
            "LD_LIBRARY_PATH=/opt/senseauto_active/senseauto-3rdparty/3rdparty/opencv/lib:/opt/senseauto_active/senseauto-3rdparty/3rdparty/lib:/opt/senseauto_active/senseauto-msgs/lib:/opt/senseauto_active/senseauto-rscl/lib"
    )

    add_executable(orsm_tracking_safety_test
        tests/orsm_tracking_safety_test.cpp
    )
    target_include_directories(orsm_tracking_safety_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(orsm_tracking_safety_test ${OpenCV_LIBS})
    add_test(NAME orsm_tracking_safety_test COMMAND orsm_tracking_safety_test)

    add_executable(orsm_zone_behavior_test
        tests/orsm_zone_behavior_test.cpp
        orsm/orsm_state_machine.cpp
        orsm/single_orsm_manager.cpp
    )
    target_include_directories(orsm_zone_behavior_test SYSTEM PRIVATE
        /opt/senseauto_active/senseauto-3rdparty/3rdparty/include
    )
    target_link_libraries(orsm_zone_behavior_test ${ORSM_RESPONSE_TEST_LIBS})
    target_link_options(orsm_zone_behavior_test PRIVATE
        "-Wl,--allow-shlib-undefined"
        "-Wl,--disable-new-dtags"
    )
    set_target_properties(orsm_zone_behavior_test PROPERTIES
        BUILD_RPATH "${SENTINEL_TEST_RUNTIME_RPATH}"
    )
    add_test(NAME orsm_zone_behavior_test COMMAND orsm_zone_behavior_test)
    set_tests_properties(orsm_zone_behavior_test PROPERTIES
        ENVIRONMENT
            "LD_LIBRARY_PATH=/opt/senseauto_active/senseauto-3rdparty/3rdparty/opencv/lib:/opt/senseauto_active/senseauto-3rdparty/3rdparty/lib:/opt/senseauto_active/senseauto-msgs/lib:/opt/senseauto_active/senseauto-rscl/lib"
    )

    add_executable(sentry_alert_manager_test
        tests/sentry_alert_manager_test.cpp
        alert/alert_manager.cpp
    )
    target_include_directories(sentry_alert_manager_test SYSTEM PRIVATE
        /opt/senseauto_active/senseauto-3rdparty/3rdparty/include
    )
    target_link_libraries(sentry_alert_manager_test ${ORSM_RESPONSE_TEST_LIBS})
    target_link_options(sentry_alert_manager_test PRIVATE
        "-Wl,--allow-shlib-undefined"
        "-Wl,--disable-new-dtags"
    )
    set_target_properties(sentry_alert_manager_test PROPERTIES
        BUILD_RPATH "${SENTINEL_TEST_RUNTIME_RPATH}"
    )
    add_test(NAME sentry_alert_manager_test COMMAND sentry_alert_manager_test)
    set_tests_properties(sentry_alert_manager_test PROPERTIES
        ENVIRONMENT
            "LD_LIBRARY_PATH=/opt/senseauto_active/senseauto-3rdparty/3rdparty/opencv/lib:/opt/senseauto_active/senseauto-3rdparty/3rdparty/lib:/opt/senseauto_active/senseauto-msgs/lib:/opt/senseauto_active/senseauto-rscl/lib"
    )
endif()
