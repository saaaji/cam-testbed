cmake_minimum_required(VERSION 3.10)
project(camera_testbed)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find udev
find_library(UDEV_LIB udev)

# find OpenCV
find_package(OpenCV REQUIRED)

# add executables
add_executable(my_app src/main.cpp src/cam.cpp)

target_include_directories(my_app PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(my_app PRIVATE ${OpenCV_LIBS} ${UDEV_LIB})