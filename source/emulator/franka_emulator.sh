#!/bin/sh
SCRIPT_PATH=$(readlink -f "$0")
SCRIPT_DIRECTORY=$(dirname "${SCRIPT_PATH}")
if [ $# != 1 ]; then
	echo 'Usage: franka_emulator.sh <IP>'
elif [ -f ${SCRIPT_DIRECTORY}/../share/franka_emulator/model/franka_emulator.world ]; then
	# Installed
	export GAZEBO_MODEL_PATH=${SCRIPT_DIRECTORY}/../share/franka_emulator/model
	export GAZEBO_PLUGIN_PATH=${SCRIPT_DIRECTORY}/../lib
	export LD_LIBRARY_PATH=${SCRIPT_DIRECTORY}/../lib
	export FRANKA_EMULATOR_IP=$1
	chrt -f 90 gazebo ${SCRIPT_DIRECTORY}/../share/franka_emulator/model/franka_emulator.world -u
elif [ -f ${SCRIPT_DIRECTORY}/model/franka_emulator.world ]; then
	# Not installed, in-directory build
	export GAZEBO_MODEL_PATH=${SCRIPT_DIRECTORY}/model
	export GAZEBO_PLUGIN_PATH=${SCRIPT_DIRECTORY}
	export LD_LIBRARY_PATH=${SCRIPT_DIRECTORY}
	export FRANKA_EMULATOR_IP=$1
	chrt -f 90 gazebo ${SCRIPT_DIRECTORY}/model/franka_emulator.world -u
elif [ -f ${SCRIPT_DIRECTORY}/../model/franka_emulator.world ]; then
	# Not installed, build in child directory
	export GAZEBO_MODEL_PATH=${SCRIPT_DIRECTORY}/../model
	export GAZEBO_PLUGIN_PATH=${SCRIPT_DIRECTORY}
	export LD_LIBRARY_PATH=${SCRIPT_DIRECTORY}
	export FRANKA_EMULATOR_IP=$1
	chrt -f 90 gazebo ${SCRIPT_DIRECTORY}/../model/franka_emulator.world -u
else
	echo "Could not find model directory"
fi
