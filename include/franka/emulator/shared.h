#pragma once

#include "../robot_state.h"
#include <pthread.h>

namespace FRANKA_EMULATOR_CXX_NAME
{
    namespace emulator
    {
        struct Shared
        {
            RobotState robot_state;
        };

        static const size_t shared_size = ((sizeof(Shared) + 4095) / 4096) * 4096;
    }
}