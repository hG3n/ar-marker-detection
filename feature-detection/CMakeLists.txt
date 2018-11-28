cmake_minimum_required(VERSION 2.8)
project(feature-detection)

# compiler flags
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-W -Wall")

find_package(OpenCV REQUIRED)
#find_package(PkgConfig REQUIRED)
# use pkg-config to find further libs

set(SOURCE_FILES
        src/main.cpp
        src/feature_detector.cpp
        src/path.cpp
        src/camera.cpp
        src/json11.cpp
        src/config.cpp)

set (HEADER_FILES "inc/")

# copy images
file(COPY images DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY calibration_images DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY config DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# libraries
set(ALL_LIBS ${OpenCV_LIBS})

# specify executable
add_executable(feature-detection ${SOURCE_FILES} ${HEADER_FILES})

# get include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${HEADER_FILES})

target_link_libraries(feature-detection ${ALL_LIBS})
