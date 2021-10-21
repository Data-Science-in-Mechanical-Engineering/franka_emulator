#!/bin/sh
export GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}
export GAZEBO_PLUGIN_PATH=${PROJECT_BINARY_DIR}
gazebo ${PROJECT_SOURCE_DIR}/model/franka_emulator.world
