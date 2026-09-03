cmake_minimum_required(VERSION 3.16)

project(UniRoboGui VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(ZMQ QUIET libzmq)
find_package(unitree_sdk2 CONFIG REQUIRED)
find_package(CURL QUIET)
find_package(OpenCV QUIET COMPONENTS core imgproc imgcodecs videoio)
find_package(realsense2 QUIET)
if(NOT realsense2_FOUND AND
   EXISTS "/opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2/realsense2Config.cmake")
    set(realsense2_DIR
        "/opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2"
        CACHE PATH "Robot image librealsense2 package" FORCE)
    find_package(realsense2 QUIET)
endif()

add_library(g1_web_core
    src/camera_service.cpp
    src/control_service.cpp
    src/g1_model_catalog.cpp
    src/http_server.cpp
    src/json_serializer.cpp
    src/joint_debug_control.cpp
    src/joint_names.cpp
    src/perception_service.cpp
    src/ros_navigation_bridge.cpp
    src/snapshot_store.cpp
    src/static_assets.cpp
    src/unitree_data_source.cpp
    src/voice_service.cpp
)

target_include_directories(g1_web_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${JSONCPP_INCLUDE_DIRS}
)

target_link_directories(g1_web_core PRIVATE ${JSONCPP_LIBRARY_DIRS})

target_link_libraries(g1_web_core
    unitree_sdk2
    Boost::system
    Boost::thread
    Threads::Threads
    ${JSONCPP_LIBRARIES}
)

target_compile_options(g1_web_core PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(
    g1_web_core PUBLIC UNI_ROBO_GUI_VERSION="${PROJECT_VERSION}")

if(CURL_FOUND)
    target_compile_definitions(g1_web_core PRIVATE G1_WEB_HAS_CURL=1)
    target_link_libraries(g1_web_core CURL::libcurl)
    message(STATUS "Customer LLM API enabled with libcurl ${CURL_VERSION_STRING}")
else()
    message(WARNING "libcurl not found: customer LLM API mode will be unavailable")
endif()

message(STATUS "SDK2-only DDS build; optional ROS 2 sidecar is disabled")

if(OpenCV_FOUND)
    target_compile_definitions(g1_web_core PRIVATE G1_WEB_HAS_OPENCV=1)
    target_include_directories(g1_web_core PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(g1_web_core ${OpenCV_LIBS})
    message(STATUS "Camera capture enabled with OpenCV ${OpenCV_VERSION}")
    if(ZMQ_FOUND)
        target_compile_definitions(g1_web_core PRIVATE G1_WEB_HAS_ZMQ=1)
        target_include_directories(g1_web_core PRIVATE ${ZMQ_INCLUDE_DIRS})
        target_link_libraries(g1_web_core ${ZMQ_LIBRARIES})
        message(STATUS "Teleimager RGB reuse enabled with libzmq ${ZMQ_VERSION}")
    else()
        message(STATUS "libzmq not found: teleimager RGB reuse disabled")
    endif()
    if(realsense2_FOUND)
        target_compile_definitions(g1_web_core PRIVATE G1_WEB_HAS_REALSENSE=1)
        target_link_libraries(g1_web_core realsense2::realsense2)
        message(STATUS "Native D435i capture enabled with librealsense2")
    else()
        message(STATUS "librealsense2 not found: --realsense will report unavailable")
    endif()
else()
    message(WARNING "OpenCV not found: camera endpoints will report unavailable")
endif()

add_executable(g1_web_server src/main.cpp)
target_link_libraries(g1_web_server PRIVATE g1_web_core)
if(realsense2_FOUND AND EXISTS "/opt/ros/noetic/lib/aarch64-linux-gnu")
    set_target_properties(g1_web_server PROPERTIES
        BUILD_RPATH "/opt/ros/noetic/lib/aarch64-linux-gnu")
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(g1_web_tests tests/test_core.cpp)
    target_include_directories(g1_web_tests PRIVATE ${JSONCPP_INCLUDE_DIRS})
    target_link_directories(g1_web_tests PRIVATE ${JSONCPP_LIBRARY_DIRS})
    target_link_libraries(g1_web_tests PRIVATE g1_web_core ${JSONCPP_LIBRARIES})
    add_test(NAME g1_web_core_tests COMMAND g1_web_tests)
    add_test(NAME g1_web_customer_config_http_tests
             COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_customer_config_http.py
                     $<TARGET_FILE:g1_web_server> ${CMAKE_CURRENT_SOURCE_DIR}/web)
endif()
