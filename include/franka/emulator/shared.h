#pragma once

#include "../robot_state.h"
#include <pthread.h>

namespace franka
{
    namespace emulator
    {
        struct Shared
        {
            RobotState robot_state;
        };
    }
}