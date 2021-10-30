#include "../include/franka_emulator/vacuum_gripper.h"

FRANKA_EMULATOR_CXX_NAME::VacuumGripper::VacuumGripper(const std::string&)
{}

FRANKA_EMULATOR_CXX_NAME::VacuumGripper::VacuumGripper(VacuumGripper&&) noexcept
{}

FRANKA_EMULATOR_CXX_NAME::VacuumGripper& FRANKA_EMULATOR_CXX_NAME::VacuumGripper::operator=(VacuumGripper&&) noexcept
{
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::VacuumGripper::~VacuumGripper() noexcept
{
}

bool FRANKA_EMULATOR_CXX_NAME::VacuumGripper::vacuum(uint8_t, std::chrono::milliseconds, ProductionSetupProfile) const
{
    return false;
}

bool FRANKA_EMULATOR_CXX_NAME::VacuumGripper::dropOff(std::chrono::milliseconds) const
{
    return false;
}

bool FRANKA_EMULATOR_CXX_NAME::VacuumGripper::stop() const
{
    return false;
}

FRANKA_EMULATOR_CXX_NAME::VacuumGripperState FRANKA_EMULATOR_CXX_NAME::VacuumGripper::readOnce() const
{
    return VacuumGripperState();
}

FRANKA_EMULATOR_CXX_NAME::VacuumGripper::ServerVersion FRANKA_EMULATOR_CXX_NAME::VacuumGripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}