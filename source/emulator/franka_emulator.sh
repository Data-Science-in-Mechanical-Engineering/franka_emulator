#!/bin/sh
if [ $# != 1 ]; then
	echo 'Usage: franka_emulator.sh <IP>'
elif [ -f $(pwd)/../share/franka_emulator/model/franka_emulator.world ]; then
	# Installed
	export GAZEBO_MODEL_PATH=$(pwd)/../share/franka_emulator/model
	export GAZEBO_PLUGIN_PATH=$(pwd)/../lib
	export FRANKA_EMULATOR_IP=$1
	chrt -f 90 gazebo $(pwd)/../share/franka_emulator/model/franka_emulator.world -u
else
	# Not installed
	export GAZEBO_MODEL_PATH=@CMAKE_SOURCE_DIR@/model
	export GAZEBO_PLUGIN_PATH=$(pwd)
	export FRANKA_EMULATOR_IP=$1
	chrt -f 90 gazebo @CMAKE_SOURCE_DIR@/model/franka_emulator.world -u
fi
