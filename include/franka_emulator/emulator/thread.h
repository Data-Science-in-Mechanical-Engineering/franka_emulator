#pragma once

#include <pthread.h>

namespace FRANKA_EMULATOR_CXX_NAME
{
    namespace emulator
    {
        class Thread
        {
        private:
            bool _attributes_created = false;
            bool _thread_created = false;
            pthread_attr_t _attributes;
            pthread_t _thread;

        public:
            Thread();
            Thread(int priority, void *data, void*(*function)(void*));
            Thread &operator=(Thread &&other);
            void join();
            ~Thread();
        };
    }
}