#include "../include/franka/control_tools.h"
#include <sys/utsname.h>
#include <pthread.h>
#include <string.h>

bool franka::hasRealtimeKernel()
{
    utsname name;
    uname(&name);
    return strstr(name.release, "rt") != nullptr;
}

bool franka::setCurrentThreadToHighestSchedulerPriority(std::string*)
{
    return pthread_setschedprio(pthread_self(), 99) == 0;
}