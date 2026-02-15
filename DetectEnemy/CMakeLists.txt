cmake_minimum_required(VERSION 3.10)
project(DetectEnemy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

# Source files
set(SOURCES
    DetectEnemy.cpp
    SerialLidar.cpp
    LidarSegmentation.cpp
)

# Create executable
add_executable(detect_enemy ${SOURCES})

# Link libraries
target_link_libraries(detect_enemy 
    ${OpenCV_LIBS}
)

# Compiler flags
if(UNIX)
    target_compile_options(detect_enemy PRIVATE -Wall -Wextra -O2)
endif()

# Installation (optional)
install(TARGETS detect_enemy DESTINATION bin)
