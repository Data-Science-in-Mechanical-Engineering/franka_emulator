#include "../include/franka/robot.h"
#include "../include/franka/model.h"
#include "../include/franka/exception.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <sched.h>
#include <fcntl.h>
#include <pthread.h>
#include <iostream> //DEBUG

franka::Robot::Robot(const std::string& franka_address, RealtimeConfig, size_t)
{
    _shared_name = std::string("/franka_emulator_") + franka_address;
    _shared_file = shm_open(_shared_name.c_str(), O_RDWR, 0644);
    if (_shared_file < 0) throw NetworkException("franka_emulator::Robot::Robot: shm_open failed");
    _shared = (emulator::Shared*) mmap(nullptr, sizeof(emulator::Shared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, _shared_file, 0);
    if (_shared == MAP_FAILED) throw NetworkException("franka_emulator::Robot::Robot: mmap failed");
}

franka::Robot::Robot(Robot&& other) noexcept
{
    _shared_name = other._shared_name;
    _shared_file = other._shared_file;
    _shared = other._shared;
    other._shared_file = -1;
    other._shared = nullptr;
    other._shared_name = "";
}

franka::Robot& franka::Robot::operator=(Robot&& other) noexcept
{
    _shared_file = other._shared_file;
    _shared = other._shared;
    other._shared_file = -1;
    other._shared = nullptr;
    return *this;
}

franka::Robot::~Robot() noexcept
{
    if (_shared != nullptr && _shared != MAP_FAILED)
    {
        munmap(_shared, sizeof(emulator::Shared));
        _shared = nullptr;
    }

    if (_shared_file >= 0)
    {
        shm_unlink(_shared_name.c_str());
        _shared_file = -1;
        _shared_name = "";
    }
}

void franka::Robot::_control()
{
    struct PthreadAttributes
    {
        pthread_attr_t data;
        PthreadAttributes() { if (pthread_attr_init(&data) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_init failed"); }
        ~PthreadAttributes() { pthread_attr_destroy(&data); }
    } pthread_attributes;    
    if (pthread_attr_setschedpolicy(&pthread_attributes.data, SCHED_FIFO) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 90;
    if (pthread_attr_setschedparam(&pthread_attributes.data, &scheduling_parameters) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_setschedpolicy failed");
    if (pthread_attr_setinheritsched(&pthread_attributes.data, PTHREAD_EXPLICIT_SCHED) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_setinheritsched failed");
    pthread_t real_time_thread;
    std::cerr << *((size_t*)&_shared->plugin_to_robot_condition) << " " << *((size_t*)&_shared->plugin_to_robot_mutex) << std::endl;
    if (pthread_create(&real_time_thread, &pthread_attributes.data, [](void* uncasted_robot) -> void*
    {
        Robot *robot = (Robot*)uncasted_robot;
        while (true)
        {
            std::cerr << "Waiting" << std::endl;
            pthread_mutex_lock(&robot->_shared->plugin_to_robot_mutex);
            std::cerr << "Waiting" << std::endl;
            pthread_cond_wait(&robot->_shared->plugin_to_robot_condition, &robot->_shared->plugin_to_robot_mutex);
            pthread_mutex_unlock(&robot->_shared->plugin_to_robot_mutex);

            std::cerr << "Signaling" << std::endl;
            pthread_mutex_lock(&robot->_shared->robot_to_plugin_mutex);
            bool finished = (*robot->_callback)(robot);
            pthread_cond_signal(&robot->_shared->robot_to_plugin_condition);
            pthread_mutex_unlock(&robot->_shared->robot_to_plugin_mutex);

            if (finished) return nullptr;
        }
    }, this) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_create failed");
    pthread_join(real_time_thread, nullptr);
}

void franka::Robot::control(std::function<Torques(const RobotState&, franka::Duration)> callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, franka::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        Torques result = context->callback(robot->_shared->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<Torques(const RobotState&, franka::Duration)> callback, std::function<JointPositions(const RobotState&, franka::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, franka::Duration)> callback;
        std::function<JointPositions(const RobotState&, franka::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<Torques(const RobotState&, franka::Duration)> callback, std::function<JointVelocities(const RobotState&, franka::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, franka::Duration)> callback;
        std::function<JointVelocities(const RobotState&, franka::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<Torques(const RobotState&, franka::Duration)> callback, std::function<CartesianPose(const RobotState&, franka::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, franka::Duration)> callback;
        std::function<CartesianPose(const RobotState&, franka::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<Torques(const RobotState&, franka::Duration)> callback, std::function<CartesianVelocities(const RobotState&, franka::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, franka::Duration)> callback;
        std::function<CartesianVelocities(const RobotState&, franka::Duration)> motion_callback;
    } context;
    context.callback = callback;
    context.motion_callback = motion_callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        context->motion_callback(robot->_shared->robot_state, Duration(1));
        Torques result = context->callback(robot->_shared->robot_state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = result.tau_J[i];
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<JointPositions(const RobotState&, franka::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<JointPositions(const RobotState&, franka::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        JointPositions result = context->callback(robot->_shared->robot_state, Duration(1));
        static const std::array<double, 7> stiffness = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
        static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = stiffness[i] * (result.q[i] - robot->_shared->robot_state.q[i]) - damping[i] * robot->_shared->robot_state.dq[i];
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<JointVelocities(const RobotState&, franka::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<JointVelocities(const RobotState&, franka::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        JointVelocities result = context->callback(robot->_shared->robot_state, Duration(1));
        static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
        for (size_t i = 0; i < 7; i++) robot->_shared->robot_state.tau_J[i] = - damping[i] * (robot->_shared->robot_state.dq[i] - result.dq[i]);
        return result.motion_finished;
    };

    _control();
}

void franka::Robot::control(std::function<CartesianPose(const RobotState&, franka::Duration)> callback, ControllerMode, bool, double)
{}

void franka::Robot::control(std::function<CartesianVelocities(const RobotState&, franka::Duration)> callback, ControllerMode, bool, double)
{}

void franka::Robot::read(std::function<bool(const RobotState&)> callback)
{}

franka::RobotState franka::Robot::readOnce()
{
    pthread_mutex_lock(&_shared->plugin_to_robot_mutex);
    pthread_cond_wait(&_shared->plugin_to_robot_condition, &_shared->plugin_to_robot_mutex);
    RobotState robot_state = _shared->robot_state;
    pthread_mutex_unlock(&_shared->plugin_to_robot_mutex);
    return robot_state;
}

franka::VirtualWallCuboid franka::Robot::getVirtualWall(int32_t id)
{
    return VirtualWallCuboid();
}

void franka::Robot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal)
{}

void franka::Robot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds,
    const std::array<double, 7>& upper_torque_thresholds,
    const std::array<double, 6>& lower_force_thresholds,
    const std::array<double, 6>& upper_force_thresholds)
{}

void franka::Robot::setJointImpedance(const std::array<double, 7>& K_theta)
{}

void franka::Robot::setCartesianImpedance(const std::array<double, 6>& K_x)
{}

void franka::Robot::setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow)
{}

void franka::Robot::setK(const std::array<double, 16>& EE_T_K)
{}

void franka::Robot::setEE(const std::array<double, 16>& NE_T_EE)
{}

void franka::Robot::setLoad(double load_mass, const std::array<double, 3>& F_x_Cload, const std::array<double, 9>& load_inertia)
{}

void franka::Robot::setFilters(
    double joint_position_filter_frequency,
    double joint_velocity_filter_frequency,
    double cartesian_position_filter_frequency,
    double cartesian_velocity_filter_frequency,
    double controller_filter_frequency)
{}

void franka::Robot::automaticErrorRecovery()
{}

void franka::Robot::stop()
{}

namespace franka
{
    class Network{};
}

franka::Model franka::Robot::loadModel()
{
    Network dummy;
    return Model(dummy);
}

franka::Robot::ServerVersion franka::Robot::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}