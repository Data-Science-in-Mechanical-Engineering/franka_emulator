#include "../include/franka/gripper_state.h"

std::ostream& operator<<(std::ostream& ostream, const franka::GripperState& gripper_state)
{
    ostream
    << "{\"width\": " << gripper_state.width
    << ", \"max_width\": " << gripper_state.max_width
    << ", \"is_grasped\": " << gripper_state.is_grasped
    << ", \"temperature\": " << gripper_state.temperature
    << ", \"time\": " << gripper_state.time.toSec()
    << "}";
    return ostream;
}