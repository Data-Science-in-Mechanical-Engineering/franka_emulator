#!/bin/sh
if [ $# != 1 ]; then
	echo 'Usage: franka_emulator.sh <IP>'
else
	export GAZEBO_MODEL_PATH=${PROJECT_SOURCE_DIR}
	export GAZEBO_PLUGIN_PATH=${PROJECT_BINARY_DIR}
	export FRANKA_EMULATOR_IP=$1
	gazebo ${PROJECT_SOURCE_DIR}/model/franka_emulator.world -u
fi
