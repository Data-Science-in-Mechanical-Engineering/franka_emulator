#pragma once

#include <string>
#include <semaphore.h>

namespace FRANKA_EMULATOR_CXX_NAME
{
    namespace emulator
    {
        class Semaphore
        {
        private:
            std::string _name;
            sem_t *_semaphore = SEM_FAILED;

        public:
            Semaphore();
            Semaphore(const std::string name, bool create, int value);
            Semaphore &operator=(Semaphore &&other);
            void close();
            void wait();
            void timedwait(int nsec);
            void post();
            void limitedpost(int limit);
            ~Semaphore();
        };
    }
}