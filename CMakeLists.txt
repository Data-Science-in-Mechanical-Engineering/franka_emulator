#################
# Configuration #
#################

cmake_minimum_required(VERSION 3.14.0)
cmake_policy(SET CMP0048 NEW)
project(franka VERSION 0.8.0)
set(FRANKA_EMULATOR_VERSION_MAJOR 0)
set(FRANKA_EMULATOR_VERSION_MINOR 0)
set(FRANKA_EMULATOR_VERSION_PATCH 0)

################
# Dependencies #
################

# c++
set(CMAKE_CXX_STANDARD 11)

# gazebo
find_package(gazebo)
if (NOT gazebo_FOUND)
	find_package(catkin REQUIRED COMPONENTS gazebo)
endif()

#pinocchio
find_package(pinocchio)
if (NOT pinocchio_FOUND)
	find_package(catkin REQUIRED COMPONENTS pinocchio)
endif()

#eigen3
find_package(Eigen3 REQUIRED)

###########
# Library #
###########

# franka_plugin
add_library(franka_plugin SHARED source/plugin.cpp)
target_compile_definitions	(franka_plugin PRIVATE FRANKA_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions	(franka_plugin PRIVATE FRANKA_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions	(franka_plugin PRIVATE FRANKA_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_compile_definitions	(franka_plugin PRIVATE FRANKA_EMULATOR_VERSION_MAJOR=${FRANKA_EMULATOR_VERSION_MAJOR})
target_compile_definitions	(franka_plugin PRIVATE FRANKA_EMULATOR_VERSION_MINOR=${FRANKA_EMULATOR_VERSION_MINOR})
target_compile_definitions	(franka_plugin PRIVATE FRANKA_EMULATOR_VERSION_PATCH=${FRANKA_EMULATOR_VERSION_PATCH})
target_include_directories	(franka_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories		(franka_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries		(franka_plugin PRIVATE ${GAZEBO_LIBRARIES})
target_compile_options		(franka_plugin PRIVATE ${GAZEBO_CXX_FLAGS})
target_include_directories	(franka_plugin PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_include_directories	(franka_plugin PRIVATE ${pinocchio_INCLUDE_DIRS})
target_link_libraries		(franka_plugin PRIVATE ${pinocchio_LIBRARIES})
target_link_libraries		(franka_plugin PRIVATE pthread rt)


# franka
add_library(franka SHARED
source/control_tools.cpp
source/control_types.cpp
source/duration.cpp
source/errors.cpp
source/exception.cpp
source/gripper.cpp
source/gripper_state.cpp
source/log.cpp
source/lowpass_filter.cpp
source/model.cpp
source/rate_limiting.cpp
source/robot.cpp
source/robot_state.cpp
source/vacuum_gripper.cpp
source/vacuum_gripper_state.cpp)
target_compile_definitions	(franka PUBLIC FRANKA_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions	(franka PUBLIC FRANKA_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions	(franka PUBLIC FRANKA_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_compile_definitions	(franka PUBLIC FRANKA_EMULATOR_VERSION_MAJOR=${FRANKA_EMULATOR_VERSION_MAJOR})
target_compile_definitions	(franka PUBLIC FRANKA_EMULATOR_VERSION_MINOR=${FRANKA_EMULATOR_VERSION_MINOR})
target_compile_definitions	(franka PUBLIC FRANKA_EMULATOR_VERSION_PATCH=${FRANKA_EMULATOR_VERSION_PATCH})
target_include_directories	(franka PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories	(franka PUBLIC ${pinocchio_INCLUDE_DIRS})
target_link_libraries		(franka PUBLIC ${pinocchio_LIBRARIES})
target_link_libraries		(franka PUBLIC pthread rt)
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_BINARY_DIR}/FrankaConfigVersion.cmake" VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
export(TARGETS franka FILE "${PROJECT_BINARY_DIR}/FrankaConfig.cmake")

#franka_emulator.sh
configure_file(source/franka_emulator.sh franka_emulator.sh)

# example
add_executable(franka_example example/example.cpp)
target_link_libraries(franka_example PRIVATE franka)
