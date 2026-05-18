cmake_minimum_required(VERSION 3.12)
project(dai_vi LANGUAGES CXX)

set(DEFAULT_BUILD_EXEC OFF)
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(DEFAULT_BUILD_EXEC ON)
endif()

find_package(depthai 3 CONFIG REQUIRED)
find_package(spdlog REQUIRED)

# Generate header with embedded python scripts
file(GLOB DEVICE_PY_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/device/*.py")
set(DEVICE_SCRIPTS_HPP "${CMAKE_CURRENT_BINARY_DIR}/generated/device_scripts.hpp")
add_custom_command(
  OUTPUT ${DEVICE_SCRIPTS_HPP}
  COMMAND ${CMAKE_COMMAND}
    -D "SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/src/device"
    -D "OUTPUT_PATH=${DEVICE_SCRIPTS_HPP}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/embed_python.cmake"
  DEPENDS ${DEVICE_PY_FILES}
  COMMENT "Embedding Python device scripts"
)
add_custom_target(dai_vi_device_scripts DEPENDS ${DEVICE_SCRIPTS_HPP})

add_library(dai_vi STATIC src/dai_vi.cpp ${DEVICE_SCRIPTS_HPP})
target_include_directories(dai_vi
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated
)
target_link_libraries(dai_vi PUBLIC depthai::core spdlog::spdlog_header_only)
set_target_properties(dai_vi PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE ON
)

option(CHECK_MSGDROP "Run checks for dropped messages on each received message" ON)
if (${CHECK_MSGDROP})
  target_compile_definitions(dai_vi PUBLIC CHECK_MSGDROP)
endif()
option(TRACE_MSGS "Log detailed information about message timing and intervals" ON)
if (${TRACE_MSGS})
  target_compile_definitions(dai_vi PUBLIC TRACE_MSGS)
endif()

option(BUILD_EXEC "Build executables for capturing and viewing" ${DEFAULT_BUILD_EXEC})
if(${BUILD_EXEC})
  message(STATUS "Building executables")

  find_package(OpenCV QUIET COMPONENTS highgui)

  include(FetchContent)
  FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
  )
  FetchContent_MakeAvailable(argparse)

  add_executable(capture src/capture.cpp)
  target_link_libraries(capture PRIVATE dai_vi argparse)

  add_executable(capture_response src/capture_response.cpp)
  target_link_libraries(capture_response PRIVATE dai_vi argparse)

  add_executable(update_imu src/update_imu.cpp)
  target_link_libraries(update_imu PRIVATE dai_vi argparse)

  if(NOT ${OpenCV_FOUND} OR NOT ${DEPTHAI_OPENCV_SUPPORT})
    message("Skipped building Viewer, OpenCV support is missing!")
  else()
    add_executable(viewer src/viewer.cpp)
    target_link_libraries(viewer PRIVATE dai_vi opencv_highgui argparse)
  endif()
endif()
