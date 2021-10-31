#include "../../include/franka_emulator/emulator/thread.h"
#include <stdexcept>

FRANKA_EMULATOR_CXX_NAME::emulator::Thread::Thread()
{}

FRANKA_EMULATOR_CXX_NAME::emulator::Thread::Thread(int priority, void *data, void*(*function)(void*))
{
    if (pthread_attr_init(&_attributes) != 0) throw std::runtime_error("franka_emulator::emulator::Thread::Thread: pthread_attr_init failed");
    _attributes_created = true;
    if (pthread_attr_setschedpolicy(&_attributes, SCHED_FIFO) != 0) throw std::runtime_error("franka_emulator::emulator::Thread::Thread: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 90;
    if (pthread_attr_setschedparam(&_attributes, &scheduling_parameters) != 0) throw std::runtime_error("franka_emulator::emulator::Thread::Thread: pthread_attr_setschedpolicy failed");
    if (pthread_attr_setinheritsched(&_attributes, PTHREAD_EXPLICIT_SCHED) != 0) throw std::runtime_error("franka_emulator::emulator::Thread::Thread: pthread_attr_setinheritsched failed");
    if (pthread_create(&_thread, &_attributes, function, data) != 0) throw std::runtime_error("franka_emulator::emulator::Thread::Thread: pthread_create failed");
    _thread_created = true;
}

FRANKA_EMULATOR_CXX_NAME::emulator::Thread &FRANKA_EMULATOR_CXX_NAME::emulator::Thread::operator=(Thread &&other)
{
    if (_attributes_created) { pthread_attr_destroy(&_attributes); _attributes_created = false; }
    _attributes_created = other._attributes_created; other._attributes_created = false;
    _thread_created = other._thread_created; other._thread_created = false;
    _attributes = other._attributes;
    _thread = other._thread;
    return *this;
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Thread::join()
{
    if (_thread_created)
    {
        pthread_join(_thread, nullptr);
        _thread_created = false;
    }
}

FRANKA_EMULATOR_CXX_NAME::emulator::Thread::~Thread()
{
    if (_attributes_created)
    {
        pthread_attr_destroy(&_attributes);
        _attributes_created = false;
    }
}