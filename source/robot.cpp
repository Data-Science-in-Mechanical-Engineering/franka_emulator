#include "../include/franka/robot.h"
#include "../include/franka/model.h"
#include "../include/franka/exception.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <sched.h>
#include <semaphore.h>

namespace FRANKA_EMULATOR_CXX_NAME
{
    class Network{};
}

FRANKA_EMULATOR_CXX_NAME::Robot::Robot(const std::string& franka_address, RealtimeConfig, size_t)
{
    //Opening shaed memory
    _ip = franka_address;
    _shared_file = shm_open(("/franka_emulator_" + _ip + "_memory").c_str(), O_RDWR, 0644);
    if (_shared_file < 0) throw NetworkException("franka_emulator::Robot::Robot: shm_open failed");
    _shared = (emulator::Shared*) mmap(nullptr, emulator::shared_size, PROT_READ | PROT_WRITE, MAP_SHARED, _shared_file, 0);
    if (_shared == MAP_FAILED) throw NetworkException("franka_emulator::Robot::Robot: mmap failed");

    //Opening semaphores
    _plugin_to_robot_mutex = sem_open(("/franka_emulator_" + _ip + "_plugin_to_robot_mutex").c_str(), 0, 0644, 0);
    if (_plugin_to_robot_mutex == SEM_FAILED) throw NetworkException("franka_emulator::Robot::Robot: sem_open failed");
    _plugin_to_robot_condition = sem_open(("/franka_emulator_" + _ip + "_plugin_to_robot_condition").c_str(), 0, 0644, 0);
    if (_plugin_to_robot_condition == SEM_FAILED) throw NetworkException("franka_emulator::Robot::Robot: sem_open failed");
    _robot_to_plugin_mutex = sem_open(("/franka_emulator_" + _ip + "_robot_to_plugin_mutex").c_str(), 0, 0644, 0);
    if (_robot_to_plugin_mutex == SEM_FAILED) throw NetworkException("franka_emulator::Robot::Robot: sem_open failed");
    _robot_to_plugin_condition = sem_open(("/franka_emulator_" + _ip + "_robot_to_plugin_condition").c_str(), 0, 0644, 0);
    if (_robot_to_plugin_condition == SEM_FAILED) throw NetworkException("franka_emulator::Robot::Robot: sem_open failed");
}

FRANKA_EMULATOR_CXX_NAME::Robot::Robot(Robot&& other) noexcept
{
    *this = std::move(other);
}

FRANKA_EMULATOR_CXX_NAME::Robot& FRANKA_EMULATOR_CXX_NAME::Robot::operator=(Robot&& other) noexcept
{
    _context                    = other._context;
    _callback                   = other._callback;
    _ip                         = other._ip;
    _shared_file                = other._shared_file;
    _plugin_to_robot_mutex      = other._plugin_to_robot_mutex;
    _plugin_to_robot_condition  = other._plugin_to_robot_condition;
    _robot_to_plugin_mutex      = other._robot_to_plugin_mutex;
    _robot_to_plugin_condition  = other._robot_to_plugin_condition;
    _shared                     = other._shared;
    other._context                  = nullptr;
    other._callback                 = nullptr;
    other._ip                       = "";
    other._shared_file              = -1;
    other._plugin_to_robot_mutex    = nullptr;
    other._plugin_to_robot_condition= nullptr;
    other._robot_to_plugin_mutex    = nullptr;
    other._robot_to_plugin_condition= nullptr;
    other._shared                   = nullptr;
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Robot::~Robot() noexcept
{
    if (_shared != nullptr && _shared != MAP_FAILED)
    {
        munmap(_shared, emulator::shared_size);
        _shared = nullptr;
    }

    if (_shared_file >= 0)
    {
        shm_unlink(("/franka_emulator_" + _ip + "_memory").c_str());
        _shared_file = -1;
    }

    if (_plugin_to_robot_mutex != nullptr && _robot_to_plugin_mutex != SEM_FAILED)
    {
        shm_unlink(("/franka_emulator_" + _ip + "_plugin_to_robot_mutex").c_str());
        _plugin_to_robot_mutex = SEM_FAILED;
    }

    if (_plugin_to_robot_condition != nullptr && _robot_to_plugin_condition != SEM_FAILED)
    {
        shm_unlink(("/franka_emulator_" + _ip + "_plugin_to_robot_condition").c_str());
        _plugin_to_robot_condition = SEM_FAILED;
    }

    if (_robot_to_plugin_mutex != nullptr && _robot_to_plugin_mutex != SEM_FAILED)
    {
        shm_unlink(("/franka_emulator_" + _ip + "_robot_to_plugin_mutex").c_str());
        _robot_to_plugin_mutex = SEM_FAILED;
    }

    if (_robot_to_plugin_condition != nullptr && _robot_to_plugin_condition != SEM_FAILED)
    {
        shm_unlink(("/franka_emulator_" + _ip + "_robot_to_plugin_condition").c_str());
        _robot_to_plugin_condition = SEM_FAILED;
    }

    _ip = "";
}

void FRANKA_EMULATOR_CXX_NAME::Robot::_control()
{
    struct PthreadAttributes
    {
        pthread_attr_t data;
        PthreadAttributes() { if (pthread_attr_init(&data) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_init failed"); }
        ~PthreadAttributes() { pthread_attr_destroy(&data); }
    } pthread_attributes;    
    if (pthread_attr_setschedpolicy(&pthread_attributes.data, SCHED_FIFO) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 80;
    if (pthread_attr_setschedparam(&pthread_attributes.data, &scheduling_parameters) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_setschedpolicy failed");
    if (pthread_attr_setinheritsched(&pthread_attributes.data, PTHREAD_EXPLICIT_SCHED) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_attr_setinheritsched failed");
    pthread_t real_time_thread;
    if (pthread_create(&real_time_thread, &pthread_attributes.data, [](void* uncasted_robot) -> void*
    {
        Robot *robot = (Robot*)uncasted_robot;
        while (true)
        {
            sem_wait(robot->_plugin_to_robot_condition);

            sem_wait(robot->_robot_to_plugin_mutex);
            bool finished; try { finished = (*robot->_callback)(robot); } //Operation also hidden behind mutex
            catch (...) { finished = true; }
            int robot_to_plugin_condition;
            sem_getvalue(robot->_robot_to_plugin_condition, &robot_to_plugin_condition);
            if (robot_to_plugin_condition == 0) sem_post(robot->_robot_to_plugin_condition);
            sem_post(robot->_robot_to_plugin_mutex);

            if (finished) return nullptr;
        }
    }, this) != 0) throw RealtimeException("franka_emulator::Robot::_control: pthread_create failed");
    pthread_join(real_time_thread, nullptr);
}

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, std::function<JointPositions(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
        std::function<JointPositions(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
        std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
        std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback, bool, double)
{
    struct Context
    {
        std::function<Torques(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
        std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> motion_callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<JointPositions(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<JointPositions(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<JointVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback;
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

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, ControllerMode, bool, double)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::control(std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR_CXX_NAME::Duration)> callback, ControllerMode, bool, double)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::read(std::function<bool(const RobotState&)> callback)
{}

FRANKA_EMULATOR_CXX_NAME::RobotState FRANKA_EMULATOR_CXX_NAME::Robot::readOnce()
{
    sem_wait(_plugin_to_robot_condition);
    sem_wait(_plugin_to_robot_mutex);
    
    RobotState robot_state = _shared->robot_state;
    sem_post(_plugin_to_robot_mutex);
    return robot_state;
}

FRANKA_EMULATOR_CXX_NAME::VirtualWallCuboid FRANKA_EMULATOR_CXX_NAME::Robot::getVirtualWall(int32_t id)
{
    return VirtualWallCuboid();
}

void FRANKA_EMULATOR_CXX_NAME::Robot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds_acceleration,
    const std::array<double, 7>& upper_torque_thresholds_acceleration,
    const std::array<double, 7>& lower_torque_thresholds_nominal,
    const std::array<double, 7>& upper_torque_thresholds_nominal,
    const std::array<double, 6>& lower_force_thresholds_acceleration,
    const std::array<double, 6>& upper_force_thresholds_acceleration,
    const std::array<double, 6>& lower_force_thresholds_nominal,
    const std::array<double, 6>& upper_force_thresholds_nominal)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setCollisionBehavior(
    const std::array<double, 7>& lower_torque_thresholds,
    const std::array<double, 7>& upper_torque_thresholds,
    const std::array<double, 6>& lower_force_thresholds,
    const std::array<double, 6>& upper_force_thresholds)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setJointImpedance(const std::array<double, 7>& K_theta)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setCartesianImpedance(const std::array<double, 6>& K_x)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setK(const std::array<double, 16>& EE_T_K)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setEE(const std::array<double, 16>& NE_T_EE)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setLoad(double load_mass, const std::array<double, 3>& F_x_Cload, const std::array<double, 9>& load_inertia)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::setFilters(
    double joint_position_filter_frequency,
    double joint_velocity_filter_frequency,
    double cartesian_position_filter_frequency,
    double cartesian_velocity_filter_frequency,
    double controller_filter_frequency)
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::automaticErrorRecovery()
{}

void FRANKA_EMULATOR_CXX_NAME::Robot::stop()
{}

FRANKA_EMULATOR_CXX_NAME::Model FRANKA_EMULATOR_CXX_NAME::Robot::loadModel()
{
    Network dummy;
    return Model(dummy);
}

FRANKA_EMULATOR_CXX_NAME::Robot::ServerVersion FRANKA_EMULATOR_CXX_NAME::Robot::serverVersion() const noexcept
{
    return 10000 * FRANKA_EMULATOR_VERSION_MAJOR + 100 * FRANKA_EMULATOR_VERSION_MINOR + FRANKA_EMULATOR_VERSION_PATCH;
}