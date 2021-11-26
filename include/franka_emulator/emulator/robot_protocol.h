#pragma once
#include "../robot_state.h"

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        struct RobotRequest
        {
            enum class Type
            {
                read,
                control,
                recovery
            };
            Type typ;
        };
        
        struct RobotResponse
        {
            enum class Type
            {
                ok,
                error
            };
            Type typ;
            RobotState state;       //Also used for control
            bool control_finished;  //Used for control
        };
    }
}
