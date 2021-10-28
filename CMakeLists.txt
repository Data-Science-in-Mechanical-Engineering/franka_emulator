#################
# Configuration #
#################

cmake_minimum_required(VERSION 3.14.0)
cmake_policy(SET CMP0048 NEW)

if (NOT DEFINED FRANKA_EMULATOR_CMAKE_NAME)
	if (DEFINED FRANKA_EMULATOR_NAME)
		set(FRANKA_EMULATOR_CMAKE_NAME ${FRANKA_EMULATOR_NAME})
	else()
		set(FRANKA_EMULATOR_CMAKE_NAME Franka)
	endif()
endif()
if (NOT DEFINED FRANKA_EMULATOR_FILE_NAME)
	if (DEFINED FRANKA_EMULATOR_NAME)
		string(TOLOWER ${FRANKA_EMULATOR_NAME} FRANKA_EMULATOR_FILE_NAME)
	else()
		set(FRANKA_EMULATOR_FILE_NAME franka)
	endif()
endif()
if (NOT DEFINED FRANKA_EMULATOR_CXX_NAME)
	if (DEFINED FRANKA_EMULATOR_NAME)
		string(TOLOWER ${FRANKA_EMULATOR_NAME} FRANKA_EMULATOR_CXX_NAME)
	else()
		set(FRANKA_EMULATOR_CXX_NAME franka)
	endif()
endif()

project(${FRANKA_EMULATOR_CMAKE_NAME} VERSION 0.8.0)
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

#eigen3
find_package(Eigen3 REQUIRED)

#pinocchio
find_package(pinocchio)
if (NOT pinocchio_FOUND)
	find_package(catkin REQUIRED COMPONENTS pinocchio)
endif()

#Google Test
find_package(GTest)

find_package(Franka)
if (NOT Franka_FOUND)
	find_package(catkin REQUIRED COMPONENTS franka_control)
endif()

###########
# Library #
###########

# franka_plugin
add_library(${FRANKA_EMULATOR_CMAKE_NAME}_plugin SHARED
source/plugin.cpp
source/model.cpp)
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_EMULATOR_CXX_NAME=${FRANKA_EMULATOR_CXX_NAME})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_EMULATOR_VERSION_MAJOR=${FRANKA_EMULATOR_VERSION_MAJOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_EMULATOR_VERSION_MINOR=${FRANKA_EMULATOR_VERSION_MINOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_EMULATOR_VERSION_PATCH=${FRANKA_EMULATOR_VERSION_PATCH})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE FRANKA_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_include_directories	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories		(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${GAZEBO_LIBRARIES})
target_compile_options		(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${GAZEBO_CXX_FLAGS})
target_include_directories	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_include_directories	(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${pinocchio_INCLUDE_DIRS})
target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE ${pinocchio_LIBRARIES})
target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PRIVATE pthread rt)
set_target_properties		(${FRANKA_EMULATOR_CMAKE_NAME}_plugin PROPERTIES OUTPUT_NAME "franka_plugin")

# franka
add_library(${FRANKA_EMULATOR_CMAKE_NAME} SHARED
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
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_EMULATOR_CXX_NAME=${FRANKA_EMULATOR_CXX_NAME})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_EMULATOR_VERSION_MAJOR=${FRANKA_EMULATOR_VERSION_MAJOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_EMULATOR_VERSION_MINOR=${FRANKA_EMULATOR_VERSION_MINOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_EMULATOR_VERSION_PATCH=${FRANKA_EMULATOR_VERSION_PATCH})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC FRANKA_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_include_directories	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_include_directories	(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC ${pinocchio_INCLUDE_DIRS})
target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC ${pinocchio_LIBRARIES})
target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME} PUBLIC pthread rt)
set_target_properties		(${FRANKA_EMULATOR_CMAKE_NAME} PROPERTIES OUTPUT_NAME ${FRANKA_EMULATOR_FILE_NAME})
include(CMakePackageConfigHelpers)
write_basic_package_version_file("${PROJECT_BINARY_DIR}/${FRANKA_EMULATOR_CMAKE_NAME}ConfigVersion.cmake" VERSION ${PROJECT_VERSION} COMPATIBILITY AnyNewerVersion)
export(TARGETS ${FRANKA_EMULATOR_CMAKE_NAME} FILE "${PROJECT_BINARY_DIR}/${FRANKA_EMULATOR_CMAKE_NAME}Config.cmake")

# example
add_executable(${FRANKA_EMULATOR_CMAKE_NAME}_example example/example.cpp)
target_link_libraries(${FRANKA_EMULATOR_CMAKE_NAME}_example PRIVATE ${FRANKA_EMULATOR_CMAKE_NAME})
set_target_properties(${FRANKA_EMULATOR_CMAKE_NAME}_example PROPERTIES OUTPUT_NAME "${FRANKA_EMULATOR_FILE_NAME}_example")

# selftest
if (Franka_FOUND AND GTest_FOUND AND NOT "${FRANKA_EMULATOR_CXX_NAME}" STREQUAL "franka")
	add_executable(${FRANKA_EMULATOR_CMAKE_NAME}_selftest example/selftest.cpp)
	target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME}_selftest PRIVATE ${FRANKA_EMULATOR_CMAKE_NAME})
	target_include_directories	(${FRANKA_EMULATOR_CMAKE_NAME}_selftest PRIVATE ${GTEST_INCLUDE_DIRS})
	target_link_directories		(${FRANKA_EMULATOR_CMAKE_NAME}_selftest PRIVATE ${GTEST_LIBRARY_DIRS})
	target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME}_selftest PRIVATE ${GTEST_LIBRARIES})
	target_link_libraries		(${FRANKA_EMULATOR_CMAKE_NAME}_selftest PRIVATE Franka::Franka)
	set_target_properties		(${FRANKA_EMULATOR_CMAKE_NAME}_selftest PROPERTIES OUTPUT_NAME "${FRANKA_EMULATOR_FILE_NAME}_selftest")
endif()

#franka_emulator.sh
configure_file(source/franka_emulator.sh franka_emulator.sh)
