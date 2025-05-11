cmake_minimum_required(VERSION 3.10)
project(camera_testbed)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find udev
find_library(UDEV_LIB udev)

# find OpenCV
find_package(OpenCV REQUIRED)

# add executables
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(cam_testbed ${SOURCES})

target_include_directories(cam_testbed PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cam_testbed PRIVATE ${OpenCV_LIBS} ${UDEV_LIB})

# >
# PYTHON LIBS
# >
file(GLOB_RECURSE PYTHON_LIB_SOURCES "tools/cmodule/*.cpp")
if (PYTHON_LIB_SOURCES)
  # find embree
  find_package(embree 3.0 REQUIRED)

  add_library(raytrace SHARED ${PYTHON_LIB_SOURCES})
  set_target_properties(raytrace PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/tools/lib
    OUTPUT_NAME "raytrace"
  )
  target_include_directories(raytrace PRIVATE ${EMBREE_INCLUDE_DIRS})
  target_link_libraries(raytrace PRIVATE ${EMBREE_LIBRARY})
else()
  message(STATUS "No source files in tools/cmodule/ directory, skipping Python bindings")
endif()