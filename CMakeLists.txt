cmake_minimum_required(VERSION 3.5)
project(dai_vi LANGUAGES CXX)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  add_compile_options(-O3 -march=native -Wall -Wextra -Wpedantic)
endif()

find_package(depthai 2.29 CONFIG REQUIRED)
find_package(spdlog REQUIRED)

add_library(dai_vi STATIC src/dai_vi.cpp)
target_include_directories(dai_vi PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(dai_vi PUBLIC depthai::core PRIVATE spdlog::spdlog)
set_target_properties(dai_vi PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  POSITION_INDEPENDENT_CODE ON
)

option(CHECK_MSGDROP "Add active checks for dropped messages" ON)
if (${CHECK_MSGDROP})
  target_compile_definitions(dai_vi PUBLIC CHECK_MSGDROP)
endif()
option(CHECK_MSGDROP_DETAILED "Add more detailed active checks for dropped messages" OFF)
if (${CHECK_MSGDROP_DETAILED})
  target_compile_definitions(dai_vi PUBLIC CHECK_MSGDROP_DETAILED)
endif()
option(DRYRUN "Run DepthAI pipeline without IO to disk" OFF)
if (${DRYRUN})
  target_compile_definitions(dai_vi PUBLIC DRYRUN)
endif()

option(BUILD_EXEC "Build executables for capturing and viewing" ON)
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
  target_link_libraries(capture PRIVATE dai_vi spdlog::spdlog argparse)

  add_executable(capture_response src/capture_response.cpp)
  target_link_libraries(capture_response PRIVATE dai_vi spdlog::spdlog argparse)

  if(NOT ${OpenCV_FOUND})
    message("Skipped building Viewer, OpenCV is missing!")
  else()
    add_executable(viewer src/viewer.cpp)
    target_link_libraries(viewer PRIVATE dai_vi depthai::opencv opencv_highgui spdlog::spdlog argparse)
  endif()
endif()
