#include "../../include/franka_emulator/emulator/semaphore.h"
#include <fcntl.h>
#include <stdexcept>

FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::Semaphore()
{}

FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::Semaphore(const std::string name, bool create, int value)
{
    _name = name;
    _semaphore = sem_open(_name.c_str(), create ? O_CREAT : 0, 0644, value);
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::Semaphore: sem_open failed");
    if (create)
    {
        int current_value;
        sem_getvalue(_semaphore, &current_value);
        while (current_value > value) { sem_wait(_semaphore); current_value--; }
        while (current_value < value) { sem_post(_semaphore); current_value++; }
    }
}

FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore &FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::operator=(Semaphore &&other)
{
    close();
    _name = other._name; other._name.clear();
    _semaphore = other._semaphore; other._semaphore = SEM_FAILED;
    return *this;
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::close()
{
    if (_semaphore != SEM_FAILED)
    {
        sem_unlink(_name.c_str());
        _name.clear();
        _semaphore = SEM_FAILED;
    }
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::wait()
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::wait: semaphore was not opened");
    sem_wait(_semaphore);
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::timedwait(int nsec)
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::wait: semaphore was not opened");
    timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_nsec += nsec;
    if (timeout.tv_nsec > 1000*1000*1000) { timeout.tv_nsec -= 1000*1000*1000; timeout.tv_sec++; }
    sem_timedwait(_semaphore, &timeout);
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::post()
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::post: semaphore was not opened");
    sem_post(_semaphore);
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::limitedpost(int limit)
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::limitedpost: semaphore was not opened");
    int value;
    sem_getvalue(_semaphore, &value);
    if (value < limit) sem_post(_semaphore);
}

FRANKA_EMULATOR_CXX_NAME::emulator::Semaphore::~Semaphore()
{
    close();
}