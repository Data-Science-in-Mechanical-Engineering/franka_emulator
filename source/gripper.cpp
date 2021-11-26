#include "../include/franka_emulator/gripper.h"

FRANKA_EMULATOR::Gripper::Gripper(const std::string& franka_address)
{
    //Opening shaed memory
    _shared = emulator::Shared("/franka_emulator_" + franka_address + "_memory", false);

    //Opening semaphores
    _request_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_gripper_request_mutex", false, 1);
    _response_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_gripper_response_mutex", false, 1);
    _request_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_gripper_request_condition", false, 0);
    _atomic_response_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_gripper_atomic_response_condition", false, 0);
    _continuous_response_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_gripper_continuous_response_condition", false, 0);
}

FRANKA_EMULATOR::Gripper::Gripper(Gripper&& gripper) noexcept
{
    *this = std::move(gripper);
}

FRANKA_EMULATOR::Gripper& FRANKA_EMULATOR::Gripper::operator=(Gripper&& gripper) noexcept
{
    std::lock_guard<std::mutex> guard1(_atomic_mutex), guard2(_continuous_mutex), guard3(gripper._atomic_mutex), guard4(gripper._continuous_mutex);
    _request_mutex                 = std::move(gripper._request_mutex);
    _response_mutex                = std::move(gripper._response_mutex);
    _request_condition             = std::move(gripper._request_condition);
    _atomic_response_condition     = std::move(gripper._atomic_response_condition);
    _continuous_response_condition = std::move(gripper._continuous_response_condition);
    _shared                        = std::move(gripper._shared);
    return *this;
}

FRANKA_EMULATOR::Gripper::~Gripper() noexcept
{
}

bool FRANKA_EMULATOR::Gripper::homing() const
{
    std::lock_guard<std::mutex> guard1(_continuous_mutex);

    //Request
    {
        std::lock_guard<std::mutex> guard2(_atomic_mutex);
        _request_mutex.wait();
        _shared.data()->gripper_request.typ = emulator::GripperRequest::Type::homing;
        _request_condition.limitedpost(1);
    }

    //Response
    _continuous_response_condition.wait();
    bool result = _shared.data()->gripper_response.typ == emulator::GripperResponse::Type::ok;
    _response_mutex.limitedpost(1);

    return result;
}

bool FRANKA_EMULATOR::Gripper::grasp(double width,
    double speed,
    double force,
    double epsilon_inner,
    double epsilon_outer) const
{
    std::lock_guard<std::mutex> guard1(_continuous_mutex);

    //Request
    {
        std::lock_guard<std::mutex> guard2(_atomic_mutex);
        _request_mutex.wait();
        _shared.data()->gripper_request.typ = emulator::GripperRequest::Type::grasp;
        _shared.data()->gripper_request.width = width;
        _shared.data()->gripper_request.speed = speed;
        _shared.data()->gripper_request.force = force;
        _shared.data()->gripper_request.epsilon_inner = epsilon_inner;
        _shared.data()->gripper_request.epsilon_outer = epsilon_outer;

        _request_condition.limitedpost(1);
    }

    //Response
    _continuous_response_condition.wait();
    bool result = _shared.data()->gripper_response.typ == emulator::GripperResponse::Type::ok;
    _response_mutex.limitedpost(1);

    return result;
}

bool FRANKA_EMULATOR::Gripper::move(double width, double speed) const
{
    std::lock_guard<std::mutex> guard1(_continuous_mutex);

    //Request
    {
        std::lock_guard<std::mutex> guard2(_atomic_mutex);
        _request_mutex.wait();
        _shared.data()->gripper_request.typ = emulator::GripperRequest::Type::move;
        _shared.data()->gripper_request.width = width;
        _shared.data()->gripper_request.speed = speed;
        _request_condition.limitedpost(1);
    }

    //Response
    _continuous_response_condition.wait();
    bool result = _shared.data()->gripper_response.typ == emulator::GripperResponse::Type::ok;
    _response_mutex.limitedpost(1);

    return result;
}

bool FRANKA_EMULATOR::Gripper::stop() const
{
    return true;
}

FRANKA_EMULATOR::GripperState FRANKA_EMULATOR::Gripper::readOnce() const
{
    std::lock_guard<std::mutex> guard(_atomic_mutex);
    
    //Request
    _request_mutex.wait();
    _shared.data()->gripper_request.typ = emulator::GripperRequest::Type::read;
    _request_condition.limitedpost(1);

    //Response
    _atomic_response_condition.wait();
    FRANKA_EMULATOR::GripperState result = _shared.data()->gripper_response.state;
    _response_mutex.limitedpost(1);
    return result;
}

FRANKA_EMULATOR::Gripper::ServerVersion FRANKA_EMULATOR::Gripper::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}