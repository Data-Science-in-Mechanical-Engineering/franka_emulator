#pragma once
#include "../gripper_state.h"

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        struct GripperRequest
        {
            enum class Type
            {
                read,
                homing,
                move,
                grasp,
                stop
            };
            Type typ;
            double width;
            double speed;
            double force;
            double epsilon_inner, epsilon_outer;
        };

        struct GripperResponse
        {
            enum class Type
            {
                ok,
                error
            };
            Type typ;
            GripperState state;
        };
    }
}