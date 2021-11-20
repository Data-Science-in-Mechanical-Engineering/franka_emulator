#include "../include/franka_emulator/robot.h"
#include "../include/franka_emulator/model.h"
#include "../include/franka_emulator/emulator/thread.h"
#include "../include/franka_emulator/emulator/network.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <sched.h>
#include <semaphore.h>

FRANKA_EMULATOR::Robot::Robot(const std::string& franka_address, RealtimeConfig, size_t)
{
    //Opening shaed memory
    _shared = emulator::Shared("/franka_emulator_" + franka_address + "_memory", false);

    //Opening semaphores
    _plugin_to_robot_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_plugin_to_robot_mutex", false, 1);
    _plugin_to_robot_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_plugin_to_robot_condition", false, 0);
    _robot_to_plugin_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_to_plugin_mutex", false, 1);
    _robot_to_plugin_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_to_plugin_condition", false, 0);
}

FRANKA_EMULATOR::Robot::Robot(Robot&& other) noexcept
{
    *this = std::move(other);
}

FRANKA_EMULATOR::Robot& FRANKA_EMULATOR::Robot::operator=(Robot&& other) noexcept
{
    _context                    = other._context;   other._context  = nullptr;
    _callback                   = other._callback;  other._callback = nullptr;
    _plugin_to_robot_mutex      = std::move(other._plugin_to_robot_mutex);
    _plugin_to_robot_condition  = std::move(other._plugin_to_robot_condition);
    _robot_to_plugin_mutex      = std::move(other._robot_to_plugin_mutex);
    _robot_to_plugin_condition  = std::move(other._robot_to_plugin_condition);
    _shared                     = std::move(other._shared);
    return *this;
}

FRANKA_EMULATOR::Robot::~Robot() noexcept
{
}

void FRANKA_EMULATOR::Robot::_control()
{
    emulator::Thread _thread(80, this, [](void* uncasted_robot) -> void*
    {
        Robot *robot = (Robot*)uncasted_robot;
        while (true)
        {
            robot->_plugin_to_robot_condition.wait();

            robot->_robot_to_plugin_mutex.wait();
            bool finished; try { finished = (*robot->_callback)(robot); } //Operation also hidden behind mutex
            catch (...) { finished = true; }
            robot->_robot_to_plugin_condition.limitedpost(1);
            robot->_robot_to_plugin_mutex.post();

            if (finished) return nullptr;
        }
    });
    _thread.join();
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
        Torques result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = result.tau_J[i];
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
        context->motion_callback(robot->_shared.data()->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = result.tau_J[i];
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
        context->motion_callback(robot->_shared.data()->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = result.tau_J[i];
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
        context->motion_callback(robot->_shared.data()->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = result.tau_J[i];
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
        context->motion_callback(robot->_shared.data()->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = result.tau_J[i];
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
        JointPositions result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        static const std::array<double, 7> stiffness = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
        static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = stiffness[i] * (result.q[i] - robot->_shared.data()->robot_state.q[i]) - damping[i] * robot->_shared.data()->robot_state.dq[i];
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
        JointVelocities result = context->callback(robot->_shared.data()->robot_state, Duration(1));
        static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_state.tau_J[i] = - damping[i] * (robot->_shared.data()->robot_state.dq[i] - result.dq[i]);
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
    _plugin_to_robot_condition.wait();
    _plugin_to_robot_mutex.wait();
    
    RobotState robot_state = _shared.data()->robot_state;
    _plugin_to_robot_mutex.post();
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
{}

void FRANKA_EMULATOR::Robot::stop()
{}

FRANKA_EMULATOR::Model FRANKA_EMULATOR::Robot::loadModel()
{
    Network dummy;
    return Model(dummy);
}

FRANKA_EMULATOR::Robot::ServerVersion FRANKA_EMULATOR::Robot::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}