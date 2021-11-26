#include "../../include/franka_emulator/emulator/semaphore.h"
#include <fcntl.h>
#include <stdexcept>

#include <errno.h>
#include <iostream>

FRANKA_EMULATOR::emulator::Semaphore::Semaphore()
{}

FRANKA_EMULATOR::emulator::Semaphore::Semaphore(const std::string name, bool create, int value)
{
    _name = name;
    _semaphore = sem_open(_name.c_str(), create ? O_CREAT : 0, 0644, value);
    if (_semaphore == SEM_FAILED) { std::cout << errno; throw std::runtime_error("franka_emulator::emulator::Semaphore::Semaphore: sem_open failed"); }
    if (create)
    {
        int current_value;
        sem_getvalue(_semaphore, &current_value);
        while (current_value > value) { sem_wait(_semaphore); current_value--; }
        while (current_value < value) { sem_post(_semaphore); current_value++; }
    }
}

FRANKA_EMULATOR::emulator::Semaphore &FRANKA_EMULATOR::emulator::Semaphore::operator=(Semaphore &&other)
{
    close();
    _name = other._name; other._name.clear();
    _semaphore = other._semaphore; other._semaphore = SEM_FAILED;
    return *this;
}

void FRANKA_EMULATOR::emulator::Semaphore::close()
{
    if (_semaphore != SEM_FAILED)
    {
        sem_close(_semaphore);
        _semaphore = SEM_FAILED;
        _name.clear();
    }
}

void FRANKA_EMULATOR::emulator::Semaphore::wait()
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::wait: semaphore was not opened");
    sem_wait(_semaphore);
}

void FRANKA_EMULATOR::emulator::Semaphore::timedwait(int nsec)
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::wait: semaphore was not opened");
    timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_nsec += nsec;
    if (timeout.tv_nsec > 1000*1000*1000) { timeout.tv_nsec -= 1000*1000*1000; timeout.tv_sec++; }
    sem_timedwait(_semaphore, &timeout);
}

void FRANKA_EMULATOR::emulator::Semaphore::post()
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::post: semaphore was not opened");
    sem_post(_semaphore);
}

void FRANKA_EMULATOR::emulator::Semaphore::limitedpost(int limit)
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::limitedpost: semaphore was not opened");
    int value;
    sem_getvalue(_semaphore, &value);
    if (value < limit) sem_post(_semaphore);
}

int FRANKA_EMULATOR::emulator::Semaphore::value()
{
    if (_semaphore == SEM_FAILED) throw std::runtime_error("franka_emulator::emulator::Semaphore::value: semaphore was not opened");
    int val;
    sem_getvalue(_semaphore, &val);
    return val;
}

FRANKA_EMULATOR::emulator::Semaphore::~Semaphore()
{
    close();
}