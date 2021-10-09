project(franka_emulator)

# dependencies
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(gazebo REQUIRED)

# franka_emulator_plugin
add_library(franka_emulator_plugin SHARED source/franka_emulator.cpp)
target_include_directories(franka_emulator_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(franka_emulator_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(franka_emulator_plugin PRIVATE ${GAZEBO_LIBRARIES})
target_compile_options(franka_emulator_plugin PRIVATE ${GAZEBO_CXX_FLAGS})