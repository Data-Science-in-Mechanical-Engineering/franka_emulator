#pragma once

#include "../robot_state.h"
#include <pthread.h>

namespace franka
{
    namespace emulator
    {
        struct Shared
        {
            pthread_cond_t plugin_to_robot_condition;
            pthread_mutex_t plugin_to_robot_mutex;
            pthread_cond_t robot_to_plugin_condition;
            pthread_mutex_t robot_to_plugin_mutex;
            RobotState robot_state;
        };
    }
}