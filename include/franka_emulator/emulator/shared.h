#pragma once

#include "robot_protocol.h"
#include "gripper_protocol.h"
#include "../robot_state.h"
#include "../gripper_state.h"
#include <sys/mman.h>
#include <string>

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        struct SharedData
        {
            RobotRequest robot_request;
            RobotResponse robot_response;

            GripperRequest gripper_request;
            GripperResponse gripper_response;
        };

        class Shared
        {
        private:
            std::string _name;
            int _file = -1;
            void *_data = MAP_FAILED;

        public:
            Shared();
            Shared(std::string name, bool create);
            Shared &operator=(Shared &&other);
            void close();
            SharedData *data();
            ~Shared();
        };
    }
}