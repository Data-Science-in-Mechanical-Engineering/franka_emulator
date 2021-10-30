#include "../include/franka_emulator/control_tools.h"
#include <sys/utsname.h>
#include <pthread.h>
#include <string.h>

bool FRANKA_EMULATOR_CXX_NAME::hasRealtimeKernel()
{
    utsname name;
    uname(&name);
    return strstr(name.release, "rt") != nullptr;
}

bool FRANKA_EMULATOR_CXX_NAME::setCurrentThreadToHighestSchedulerPriority(std::string*)
{
    return pthread_setschedprio(pthread_self(), 99) == 0;
}