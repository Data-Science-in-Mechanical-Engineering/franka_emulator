#pragma once

#include "../robot_state.h"
#include <sys/mman.h>
#include <string>

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        struct SharedData
        {
            RobotState robot_state;
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