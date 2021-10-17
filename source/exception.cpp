#include "../include/franka/exception.h"

franka::IncompatibleVersionException::IncompatibleVersionException(uint16_t server_version, uint16_t library_version) noexcept :
    server_version(server_version),
    library_version(library_version),
    Exception("franka_emulator::IncompatibleVersionException")
{
}

franka::ControlException::ControlException(const std::string& what, std::vector<franka::Record> log) noexcept :
    log(std::move(log)),
    Exception(what)
{
}