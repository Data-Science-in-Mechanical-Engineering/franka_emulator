#include "../include/franka_emulator/exception.h"

FRANKA_EMULATOR_CXX_NAME::IncompatibleVersionException::IncompatibleVersionException(uint16_t server_version, uint16_t library_version) noexcept :
    server_version(server_version),
    library_version(library_version),
    Exception("franka_emulator::IncompatibleVersionException")
{
}

FRANKA_EMULATOR_CXX_NAME::ControlException::ControlException(const std::string& what, std::vector<FRANKA_EMULATOR_CXX_NAME::Record> log) noexcept :
    log(std::move(log)),
    Exception(what)
{
}