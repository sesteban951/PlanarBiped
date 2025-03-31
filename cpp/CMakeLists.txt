cmake_minimum_required(VERSION 3.10)

# Project name
project(biped)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O3 -pthread")

# Find GLFW
find_package(glfw3 REQUIRED)

# Include Eigen headers
include_directories($ENV{EIGEN_DIR})

# Include MuJoCo
include_directories($ENV{MUJOCO_DIR}/include)
include_directories($ENV{MUJOCO_DIR}/include/mujoco)
find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib REQUIRED)

# Include project header files
include_directories(include)

# Add the controller library
add_library(controller SHARED src/controller.cpp)
target_include_directories(controller PUBLIC include)
target_link_libraries(controller PUBLIC glfw ${MUJOCO_LIB})

# Add the simulator library
add_library(simulator SHARED src/simulator.cpp)
target_include_directories(simulator PUBLIC include)
target_link_libraries(simulator PUBLIC glfw ${MUJOCO_LIB})

# Add the logger library
add_library(logger SHARED src/logger.cpp)
target_include_directories(logger PUBLIC include)
target_link_libraries(logger PUBLIC glfw ${MUJOCO_LIB})

# Main executable
add_executable(demo main.cpp)
target_include_directories(demo PUBLIC include)
target_link_libraries(demo controller simulator logger glfw ${MUJOCO_LIB})