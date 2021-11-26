#include "../include/franka_emulator/robot.h"
#include "../include/franka_emulator/model.h"
#include "../include/franka_emulator/exception.h"
#include "../include/franka_emulator/emulator/thread.h"
#include "../include/franka_emulator/emulator/network.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <sched.h>
#include <semaphore.h>

FRANKA_EMULATOR::Robot::Robot(const std::string& franka_address, RealtimeConfig, size_t) : _command_running(true)
{
    _shared = emulator::Shared("/franka_emulator_" + franka_address + "_memory", false);
    _request_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_request_mutex", false, 1);
    _response_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_response_mutex", false, 1);
    _request_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_request_condition", false, 0);
    _response_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_response_condition", false, 0);
    _control_call_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_control_call_condition", false, 0);
    _control_return_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_control_return_condition", false, 0);
    _command_running = false;
}

FRANKA_EMULATOR::Robot::Robot(Robot&& other) noexcept
{
    *this = std::move(other);
}

FRANKA_EMULATOR::Robot& FRANKA_EMULATOR::Robot::operator=(Robot&& other) noexcept
{
    _command_running              .store(other._command_running);
    _control_running              = other._control_running;
    _callback                     = other._callback;
    _context                      = other._context;
    _request_mutex                = std::move(other._request_mutex);
    _response_mutex               = std::move(other._response_mutex);
    _request_condition            = std::move(other._request_condition);
    _response_condition           = std::move(other._response_condition);
    _control_call_condition       = std::move(other._control_call_condition);
    _control_return_condition     = std::move(other._control_return_condition);
    _shared                       = std::move(other._shared);
    return *this;
}

FRANKA_EMULATOR::Robot::~Robot() noexcept
{
}

void FRANKA_EMULATOR::Robot::_control()
{
    bool expected = false;
    if (!_command_running.compare_exchange_strong(expected, true)) throw ControlException("Command already running");
    _control_running = true;

    //Sending reuest
    _request_mutex.wait();
    _shared.data()->robot_request.typ = emulator::RobotRequest::Type::control;
    _request_condition.limitedpost(1);

    //Waiting response
    _response_condition.wait();
    _response_mutex.limitedpost(1);

    //Handling control calls
    emulator::Thread _thread(80, this, [](void* uncasted_robot) -> void*
    {
        Robot *robot = (Robot*)uncasted_robot;
        while (true)
        {
            robot->_control_call_condition.wait();
            bool finished;
            if (robot->_shared.data()->robot_response.state.current_errors) finished = true; //Server sent error
            else if (robot->_control_running == false) finished = true;                      //Stop() was called
            else try { finished = (*robot->_callback)(robot); }                              //Callback set motion_finished
            catch (...) { finished = true; }                                                 //Callback threw exception
            robot->_control_return_condition.limitedpost(1);
            if (finished) return nullptr;
        }
    });
    _thread.join();

    _control_running = false;
    _command_running = false;
}

void FRANKA_EMULATOR::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        Torques result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback, std::function<JointPositions(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
        std::function<JointPositions(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared.data()->robot_response.state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback, std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
        std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared.data()->robot_response.state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback, std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
        std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared.data()->robot_response.state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback, std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
        std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared.data()->robot_response.state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<JointPositions(const RobotState&, FRANKA_EMULATOR::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<JointPositions(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        JointPositions result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        static const std::array<double, 7> stiffness = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
        static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = stiffness[i] * (result.q[i] - robot->_shared.data()->robot_response.state.q[i]) - damping[i] * robot->_shared.data()->robot_response.state.dq[i];
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        JointVelocities result = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = - damping[i] * (robot->_shared.data()->robot_response.state.dq[i] - result.dq[i]);
        return result.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> callback, ControllerMode, bool, double)
{}

void FRANKA_EMULATOR::Robot::control(std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> callback, ControllerMode, bool, double)
{}

void FRANKA_EMULATOR::Robot::read(std::function<bool(const RobotState&)> callback)
{}

FRANKA_EMULATOR::RobotState FRANKA_EMULATOR::Robot::readOnce()
{
    bool expected = false;
    if (!_command_running.compare_exchange_strong(expected, true)) throw ControlException("Command already running");
    
    //Request
    _request_mutex.wait();
    _shared.data()->robot_request.typ = emulator::RobotRequest::Type::read;
    _request_condition.limitedpost(1);

    //Response
    _response_condition.wait();
    RobotState robot_state = _shared.data()->robot_response.state;
    _response_mutex.limitedpost(1);

    _command_running = false;
    return robot_state;
}

FRANKA_EMULATOR::VirtualWallCuboid FRANKA_EMULATOR::Robot::getVirtualWall(int32_t id)
{
    return VirtualWallCuboid();
}

void FRANKA_EMULATOR::Robot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal)
{}

void FRANKA_EMULATOR::Robot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds,
    const std::array<double, 7>& upper_torque_thresholds,
    const std::array<double, 6>& lower_force_thresholds,
    const std::array<double, 6>& upper_force_thresholds)
{}

void FRANKA_EMULATOR::Robot::setJointImpedance(const std::array<double, 7>& K_theta)
{}

void FRANKA_EMULATOR::Robot::setCartesianImpedance(const std::array<double, 6>& K_x)
{}

void FRANKA_EMULATOR::Robot::setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow)
{}

void FRANKA_EMULATOR::Robot::setK(const std::array<double, 16>& EE_T_K)
{}

void FRANKA_EMULATOR::Robot::setEE(const std::array<double, 16>& NE_T_EE)
{}

void FRANKA_EMULATOR::Robot::setLoad(double load_mass, const std::array<double, 3>& F_x_Cload, const std::array<double, 9>& load_inertia)
{}

void FRANKA_EMULATOR::Robot::setFilters(
    double joint_position_filter_frequency,
    double joint_velocity_filter_frequency,
    double cartesian_position_filter_frequency,
    double cartesian_velocity_filter_frequency,
    double controller_filter_frequency)
{}

void FRANKA_EMULATOR::Robot::automaticErrorRecovery()
{
    bool expected = false;
    if (!_command_running.compare_exchange_strong(expected, true)) throw ControlException("Command already running");

    //Request
    _request_mutex.wait();
    _shared.data()->robot_request.typ = emulator::RobotRequest::Type::recovery;
    _request_condition.limitedpost(1);

    //Response
    _response_condition.wait();
    _response_mutex.limitedpost(1);
    
    _command_running = false;
}

void FRANKA_EMULATOR::Robot::stop()
{
    _command_running = false;
}

FRANKA_EMULATOR::Model FRANKA_EMULATOR::Robot::loadModel()
{
    Network dummy;
    return Model(dummy);
}

FRANKA_EMULATOR::Robot::ServerVersion FRANKA_EMULATOR::Robot::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}