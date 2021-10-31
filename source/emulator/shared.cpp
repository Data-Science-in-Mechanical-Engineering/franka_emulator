#include "../../include/franka_emulator/emulator/shared.h"
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdexcept>

FRANKA_EMULATOR_CXX_NAME::emulator::Shared::Shared()
{}

FRANKA_EMULATOR_CXX_NAME::emulator::Shared::Shared(std::string name, bool create)
{
    _file = shm_open(name.c_str(), create ? (O_CREAT | O_RDWR) : O_RDWR, 0644);
    if (_file < 0) throw std::runtime_error("franka_emulator::emulator::Shared::open: shm_open failed");
    if (ftruncate(_file, sizeof(SharedData)) != 0) throw std::runtime_error("franka_emulator::emulator::Shared::open: ftruncate failed");
    _data = (SharedData*) mmap(nullptr, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, _file, 0);
    if (_data == MAP_FAILED) throw std::runtime_error("franka_emulator::emulator::Shared::open: mmap failed");
    memset(_data, 0, sizeof(SharedData));
}

FRANKA_EMULATOR_CXX_NAME::emulator::Shared &FRANKA_EMULATOR_CXX_NAME::emulator::Shared::operator=(Shared &&other)
{
    close();
    _name = other._name; other._name.clear();
    _file = other._file; other._file = -1;
    _data = other._data; other._data = MAP_FAILED;
    return *this;
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Shared::close()
{
    if (_data != MAP_FAILED) { munmap(_data, sizeof(SharedData)); _data = MAP_FAILED; }
    if (_file >= 0) { shm_unlink(_name.c_str()); _file = -1; }
    _name.clear();
}

FRANKA_EMULATOR_CXX_NAME::emulator::SharedData *FRANKA_EMULATOR_CXX_NAME::emulator::Shared::data()
{
    if (_data == MAP_FAILED) throw std::runtime_error("franka_emulator::emulator::Shared::data: shared memory was not opened");
    return (SharedData*)_data;
}

FRANKA_EMULATOR_CXX_NAME::emulator::Shared::~Shared()
{
    close();
}