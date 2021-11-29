#################
# Configuration #
#################

cmake_minimum_required(VERSION 3.4)
cmake_policy(SET CMP0048 NEW)
include(CMakePackageConfigHelpers)

if(FRANKA_EMULATOR_IMITATE)
  project(libfranka VERSION 0.8.0)
  set(FRANKA_EMULATOR_NAME_UPPERCASE Franka)
  set(FRANKA_EMULATOR_NAME_LOWERCASE franka)
  if (NOT EXISTS ${CMAKE_SOURCE_DIR}/include/franka)
    execute_process(COMMAND ln -s ${CMAKE_SOURCE_DIR}/include/franka_emulator ${CMAKE_SOURCE_DIR}/include/franka)
  endif()

else()
  project(libfranka_emulator VERSION 1.0.0)
  set(FRANKA_EMULATOR_NAME_UPPERCASE FrankaEmulator)
  set(FRANKA_EMULATOR_NAME_LOWERCASE franka_emulator)
  if (EXISTS ${CMAKE_SOURCE_DIR}/include/franka)
    execute_process(COMMAND rm -rf ${CMAKE_SOURCE_DIR}/include/franka)
  endif()
endif()

if (${CMAKE_SOURCE_DIR}/model/franka_emulator/model.urdf IS_NEWER_THAN ${CMAKE_SOURCE_DIR}/model/franka_emulator/model.sdf)
  execute_process(COMMAND gz sdf -p ${CMAKE_SOURCE_DIR}/model/franka_emulator/model.urdf OUTPUT_FILE ${CMAKE_SOURCE_DIR}/model/franka_emulator/model.sdf)
endif()

set(FRANKA_EMULATOR_VERSION_MAJOR 1)
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

# eigen3
find_package(Eigen3 REQUIRED)

# pinocchio
find_package(pinocchio)
if (NOT pinocchio_FOUND)
  find_package(catkin REQUIRED COMPONENTS pinocchio)
endif()

# Google Test
find_package(GTest)

# libfranka
if (NOT FRANKA_EMULATOR_IMITATE)
  find_package(Franka)
  if (NOT Franka_FOUND)
    find_package(catkin COMPONENTS franka_control)
  endif()
endif()

###########
# Library #
###########

# franka_emulator_model
add_library                   (franka_emulator_model SHARED source/model.cpp)
target_compile_definitions    (franka_emulator_model PUBLIC FRANKA_EMULATOR=${FRANKA_EMULATOR_NAME_LOWERCASE})
target_include_directories    (franka_emulator_model PUBLIC ${pinocchio_INCLUDE_DIRS})
target_link_libraries         (franka_emulator_model PUBLIC ${pinocchio_LIBRARIES})
target_include_directories    (franka_emulator_model PUBLIC ${EIGEN3_INCLUDE_DIRS})
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  target_compile_options      (franka_emulator_model PRIVATE -O3)
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_compile_options      (franka_emulator_model PRIVATE /O2)
endif()

# franka_emulator
add_library(franka_emulator SHARED
source/emulator/semaphore.cpp
source/emulator/shared.cpp
source/emulator/thread.cpp
source/control_tools.cpp
source/control_types.cpp
source/duration.cpp
source/errors.cpp
source/exception.cpp
source/gripper.cpp
source/gripper_state.cpp
source/log.cpp
source/lowpass_filter.cpp
source/rate_limiting.cpp
source/robot.cpp
source/robot_state.cpp
source/vacuum_gripper.cpp
source/vacuum_gripper_state.cpp)
if (FRANKA_EMULATOR_IMITATE)
  target_compile_definitions  (franka_emulator PUBLIC FRANKA_EMULATOR_IMITATE)
endif()
target_compile_definitions    (franka_emulator PUBLIC FRANKA_EMULATOR_VERSION_MAJOR=${FRANKA_EMULATOR_VERSION_MAJOR})
target_compile_definitions    (franka_emulator PUBLIC FRANKA_EMULATOR_VERSION_MINOR=${FRANKA_EMULATOR_VERSION_MINOR})
target_compile_definitions    (franka_emulator PUBLIC FRANKA_EMULATOR_VERSION_PATCH=${FRANKA_EMULATOR_VERSION_PATCH})
target_compile_definitions    (franka_emulator PUBLIC FRANKA_VERSION_MAJOR=${PROJECT_VERSION_MAJOR})
target_compile_definitions    (franka_emulator PUBLIC FRANKA_VERSION_MINOR=${PROJECT_VERSION_MINOR})
target_compile_definitions    (franka_emulator PUBLIC FRANKA_VERSION_PATCH=${PROJECT_VERSION_PATCH})
target_include_directories    (franka_emulator PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries         (franka_emulator PUBLIC franka_emulator_model)

# franka_emulator_plugin
add_library                   (franka_emulator_plugin SHARED source/emulator/plugin.cpp)
target_link_libraries         (franka_emulator_plugin PRIVATE franka_emulator)
target_include_directories    (franka_emulator_plugin PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories       (franka_emulator_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries         (franka_emulator_plugin PRIVATE ${GAZEBO_LIBRARIES})
target_compile_options        (franka_emulator_plugin PRIVATE ${GAZEBO_CXX_FLAGS})

# franka_emulator_inertia_calibrator
if (Franka_FOUND)
  add_executable              (franka_emulator_inertia_calibrator source/emulator/inertia_calibrator.cpp)
  target_link_libraries       (franka_emulator_inertia_calibrator PRIVATE franka_emulator)
  target_link_libraries       (franka_emulator_inertia_calibrator PRIVATE Franka::Franka)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    target_compile_options    (franka_emulator_inertia_calibrator PRIVATE -O3)
  elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    target_compile_options    (franka_emulator_inertia_calibrator PRIVATE /O2)
  endif()
endif()

# franka_emulator_friction_calibrator
if (Franka_FOUND)
  add_executable              (franka_emulator_friction_calibrator source/emulator/friction_calibrator.cpp)
  target_link_libraries       (franka_emulator_friction_calibrator PRIVATE franka_emulator)
  target_link_libraries       (franka_emulator_friction_calibrator PRIVATE Franka::Franka)
endif()

# example
add_executable                (franka_emulator_example example/example.cpp)
target_link_libraries         (franka_emulator_example PRIVATE franka_emulator)

# selftest
if (GTest_FOUND)
  add_executable              (franka_emulator_selftest example/selftest.cpp)
  target_link_libraries       (franka_emulator_selftest PRIVATE franka_emulator)
  if (Franka_FOUND)
    target_compile_definitions(franka_emulator_selftest PRIVATE Franka_FOUND)
    target_link_libraries     (franka_emulator_selftest PRIVATE Franka::Franka)
  endif()
  target_include_directories  (franka_emulator_selftest PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_directories     (franka_emulator_selftest PRIVATE ${GTEST_LIBRARY_DIRS})
  target_link_libraries       (franka_emulator_selftest PRIVATE ${GTEST_LIBRARIES})
endif()

# franka_emulator.sh
configure_file                (source/emulator/franka_emulator.sh franka_emulator.sh COPYONLY)

# export
add_library                      (Franka::${FRANKA_EMULATOR_NAME_UPPERCASE} ALIAS franka_emulator)
set_target_properties            (franka_emulator PROPERTIES EXPORT_NAME ${FRANKA_EMULATOR_NAME_UPPERCASE})
export                           (TARGETS franka_emulator_model franka_emulator franka_emulator_plugin NAMESPACE Franka:: FILE ${CMAKE_BINARY_DIR}/${FRANKA_EMULATOR_NAME_UPPERCASE}Targets.cmake)

# installation
configure_package_config_file    (FrankaEmulatorConfig.cmake.in ${CMAKE_BINARY_DIR}/${FRANKA_EMULATOR_NAME_UPPERCASE}Config.cmake INSTALL_DESTINATION lib/cmake/${FRANKA_EMULATOR_NAME_UPPERCASE})
write_basic_package_version_file (${CMAKE_BINARY_DIR}/${FRANKA_EMULATOR_NAME_UPPERCASE}ConfigVersion.cmake COMPATIBILITY SameMajorVersion)
install                          (TARGETS franka_emulator_model franka_emulator franka_emulator_plugin EXPORT ${FRANKA_EMULATOR_NAME_UPPERCASE}Targets)
install                          (EXPORT ${FRANKA_EMULATOR_NAME_UPPERCASE}Targets NAMESPACE Franka:: DESTINATION lib/cmake/${FRANKA_EMULATOR_NAME_UPPERCASE})
install                          (FILES ${CMAKE_BINARY_DIR}/${FRANKA_EMULATOR_NAME_UPPERCASE}Config.cmake ${CMAKE_BINARY_DIR}/${FRANKA_EMULATOR_NAME_UPPERCASE}ConfigVersion.cmake DESTINATION lib/cmake/${FRANKA_EMULATOR_NAME_UPPERCASE})
install                          (DIRECTORY ${PROJECT_SOURCE_DIR}/include/franka_emulator/ DESTINATION include/${FRANKA_EMULATOR_NAME_LOWERCASE})
install                          (DIRECTORY ${PROJECT_SOURCE_DIR}/model/ DESTINATION share/franka_emulator/model)
install                          (PROGRAMS ${CMAKE_BINARY_DIR}/franka_emulator.sh DESTINATION bin)
