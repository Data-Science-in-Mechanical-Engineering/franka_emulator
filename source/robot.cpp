#include "../include/franka_emulator/robot.h"
#include "../include/franka_emulator/model.h"
#include "../include/franka_emulator/exception.h"
#include "../include/franka_emulator/emulator/thread.h"
#include "../include/franka_emulator/emulator/constants.h"
#include <Eigen/Geometry>
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <sched.h>
#include <semaphore.h>

FRANKA_EMULATOR::Robot::Robot(const std::string& franka_address, RealtimeConfig, size_t) : _model(_network)
{
    _cartesian_stiffness.segment<3>(0) = emulator::default_translational_stiffness * Eigen::Matrix<double, 3, 1>::Ones();
    _cartesian_stiffness.segment<3>(3) = emulator::default_rotational_stiffness * Eigen::Matrix<double, 3, 1>::Ones();
    _cartesian_damping.array() = 2 * _cartesian_damping.array().sqrt();
    _joint_stiffness = Eigen::Matrix<double, 7, 1>::Map(emulator::default_joint_stiffness.data());
    _joint_damping = Eigen::Matrix<double, 7, 1>::Map(emulator::default_joint_damping.data());
    
    _command_running.store(false);
    _control_running = false;
    _shared = emulator::Shared("/franka_emulator_" + franka_address + "_memory", false);
    _request_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_request_mutex", false, 1);
    _response_mutex = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_response_mutex", false, 1);
    _request_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_request_condition", false, 0);
    _response_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_response_condition", false, 0);
    _control_call_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_control_call_condition", false, 0);
    _control_return_condition = emulator::Semaphore("/franka_emulator_" + franka_address + "_robot_control_return_condition", false, 0);
}

FRANKA_EMULATOR::Robot::Robot(Robot&& other) noexcept : _model(std::move(other._model))
{
    *this = std::move(other);
}

FRANKA_EMULATOR::Robot& FRANKA_EMULATOR::Robot::operator=(Robot&& other) noexcept
{
    _cartesian_stiffness        = other._cartesian_stiffness;
    _cartesian_damping          = other._cartesian_damping;
    _joint_stiffness            = other._joint_stiffness;
    _joint_damping              = other._joint_damping;

    _command_running            .store(other._command_running);
    _control_running            = other._control_running;
    _callback                   = other._callback;
    _context                    = other._context;
    _request_mutex              = std::move(other._request_mutex);
    _response_mutex             = std::move(other._response_mutex);
    _request_condition          = std::move(other._request_condition);
    _response_condition         = std::move(other._response_condition);
    _control_call_condition     = std::move(other._control_call_condition);
    _control_return_condition   = std::move(other._control_return_condition);
    _shared                     = std::move(other._shared);
    return *this;
}

FRANKA_EMULATOR::Robot::~Robot() noexcept
{
}

void FRANKA_EMULATOR::Robot::_control()
{
    bool expected = false;
    if (!_command_running.compare_exchange_strong(expected, true)) throw ControlException("franka_emulator::Robot::_control: Command already running");
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
        robot->_shared.data()->robot_response.state.tau_J = result.tau_J;
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
        robot->_shared.data()->robot_response.state.tau_J = result.tau_J;
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
        robot->_shared.data()->robot_response.state.tau_J = result.tau_J;
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
        robot->_shared.data()->robot_response.state.tau_J = result.tau_J;
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
        robot->_shared.data()->robot_response.state.tau_J = result.tau_J;
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
        JointPositions target = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] =
            + robot->_joint_stiffness(i) * (target.q[i] - robot->_shared.data()->robot_response.state.q[i])
            + robot->_joint_damping(i) * (-robot->_shared.data()->robot_response.state.dq[i]);
        return target.motion_finished;
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
        JointVelocities target = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] =
            + robot->_joint_damping(i) * (target.dq[i] - robot->_shared.data()->robot_response.state.dq[i]);
        return target.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<CartesianPose(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        CartesianPose target = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        Eigen::Affine3d target_transform(Eigen::Matrix<double, 4, 4>::Map(target.O_T_EE.data()));
        Eigen::Quaterniond target_orientation(target_transform.linear());
        Eigen::Affine3d current_transform(Eigen::Matrix<double, 4, 4>::Map(robot->_shared.data()->robot_response.state.O_T_EE.data()));
        Eigen::Quaterniond current_orientation(current_transform.linear());
        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
        error.segment<3>(0) = current_transform.translation() -  target_transform.translation();
        if (target_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) current_orientation.coeffs() << -current_orientation.coeffs();
        Eigen::Quaterniond error_quaternion(current_orientation.inverse() * target_orientation);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        error.tail(3) << -current_transform.linear() * error.tail(3);
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(robot->_model.coriolis(robot->_shared.data()->robot_response.state).data());
        Eigen::Matrix<double, 6, 7> stiffness_jacobian = Eigen::Matrix<double, 6, 7>::Map(robot->_model.zeroJacobian(franka::Frame::kStiffness, robot->_shared.data()->robot_response.state).data());
        Eigen::Matrix<double, 6, 1> velocity = stiffness_jacobian * Eigen::Matrix<double, 7, 1>::Map(robot->_shared.data()->robot_response.state.dq.data());
        Eigen::Matrix<double, 7, 1> torques = coriolis + stiffness_jacobian.transpose() * (
            - robot->_cartesian_stiffness.array() * error.array()
            - robot->_cartesian_damping.array() * velocity.array()
        ).matrix();
        Eigen::Matrix<double, 7, 1>::Map(&robot->_shared.data()->robot_response.state.tau_J[0]) = torques;
        return target.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::control(std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> callback, ControllerMode, bool, double)
{
    struct Context
    {
        std::function<CartesianVelocities(const RobotState&, FRANKA_EMULATOR::Duration)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        CartesianVelocities target = context->callback(robot->_shared.data()->robot_response.state, Duration(1));
        Eigen::Matrix<double, 6, 1> target_velocity = Eigen::Matrix<double, 6, 1>::Map(target.O_dP_EE.data());
        Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Map(robot->_model.coriolis(robot->_shared.data()->robot_response.state).data());
        Eigen::Matrix<double, 6, 7> stiffness_jacobian = Eigen::Matrix<double, 6, 7>::Map(robot->_model.zeroJacobian(franka::Frame::kStiffness, robot->_shared.data()->robot_response.state).data());
        Eigen::Matrix<double, 6, 1> velocity = stiffness_jacobian * Eigen::Matrix<double, 7, 1>::Map(robot->_shared.data()->robot_response.state.dq.data());
        Eigen::Matrix<double, 7, 1> torques = coriolis + stiffness_jacobian.transpose() * (
            - robot->_cartesian_damping.array() * (velocity - target_velocity).array()
        ).matrix();
        Eigen::Matrix<double, 7, 1>::Map(&robot->_shared.data()->robot_response.state.tau_J[0]) = torques;
        return target.motion_finished;
    };

    _control();
}

void FRANKA_EMULATOR::Robot::read(std::function<bool(const RobotState&)> callback)
{
    struct Context
    {
        std::function<bool(const RobotState&)> callback;
    } context;
    context.callback = callback;
    _context = &context;

    _callback = [](Robot* robot) -> bool
    {
        Context *context = (Context*)robot->_context;
        bool run = context->callback(robot->_shared.data()->robot_response.state);
        for (size_t i = 0; i < 7; i++) robot->_shared.data()->robot_response.state.tau_J[i] = 0.0;
        return run;
    };

    _control();
}

FRANKA_EMULATOR::RobotState FRANKA_EMULATOR::Robot::readOnce()
{
    bool expected = false;
    if (!_command_running.compare_exchange_strong(expected, true)) throw ControlException("franka_emulator::Robot::readOnce: Command already running");
    
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
{
    _joint_stiffness = Eigen::Matrix<double, 7, 1>::Map(K_theta.data());
    _joint_damping.array() = 2.0 * _joint_stiffness.array().sqrt() *
        Eigen::Matrix<double, 7, 1>::Map(emulator::default_joint_damping.data()).array() / (2.0 * Eigen::Matrix<double, 7, 1>::Map(emulator::default_joint_stiffness.data()).array().sqrt());
}

void FRANKA_EMULATOR::Robot::setCartesianImpedance(const std::array<double, 6>& K_x)
{
    _cartesian_stiffness = Eigen::Matrix<double, 6, 1>::Map(K_x.data());
    _cartesian_damping.array() = 2 * _cartesian_damping.array().sqrt();   
}

void FRANKA_EMULATOR::Robot::setGuidingMode(const std::array<bool, 6>& guiding_mode, bool elbow)
{}

void FRANKA_EMULATOR::Robot::setK(const std::array<double, 16>& EE_T_K)
{
    _shared.data()->robot_response.state.EE_T_K = EE_T_K;
}

void FRANKA_EMULATOR::Robot::setEE(const std::array<double, 16>& NE_T_EE)
{
    Eigen::Matrix<double, 4, 4>::Map(&_shared.data()->robot_response.state.F_T_EE[0]) =
        Eigen::Matrix<double, 4, 4>::Map(&emulator::nominal_gripper_frame[0]).inverse() * Eigen::Matrix<double, 4, 4>::Map(&NE_T_EE[0]);
    _shared.data()->robot_response.state.NE_T_EE = NE_T_EE;
}

void FRANKA_EMULATOR::Robot::setLoad(double load_mass, const std::array<double, 3>& F_x_Cload, const std::array<double, 9>& load_inertia)
{
    _shared.data()->robot_response.state.m_load = load_mass;
    _shared.data()->robot_response.state.F_x_Cload = F_x_Cload;
    _shared.data()->robot_response.state.I_load = load_inertia;
    
    _shared.data()->robot_response.state.m_total = _shared.data()->robot_response.state.m_ee + _shared.data()->robot_response.state.m_load;
    for (size_t i = 0; i < 3; i++) _shared.data()->robot_response.state.F_x_Ctotal[i] = (
        _shared.data()->robot_response.state.m_ee * _shared.data()->robot_response.state.F_x_Cee[i] +
        _shared.data()->robot_response.state.m_load * _shared.data()->robot_response.state.F_x_Cload[i]
    ) / _shared.data()->robot_response.state.m_total;
    for (size_t i = 0; i < 3; i++) for (size_t j = 0; j < 3; j++) _shared.data()->robot_response.state.I_total[3*i+j] = (
        _shared.data()->robot_response.state.I_ee[3*i+j] +
        _shared.data()->robot_response.state.m_ee * (_shared.data()->robot_response.state.F_x_Ctotal[i] - _shared.data()->robot_response.state.F_x_Cee[i]) * (_shared.data()->robot_response.state.F_x_Ctotal[j] - _shared.data()->robot_response.state.F_x_Cee[j]) +
        _shared.data()->robot_response.state.I_load[3*i+j] +
        _shared.data()->robot_response.state.m_load * (_shared.data()->robot_response.state.F_x_Ctotal[i] - _shared.data()->robot_response.state.F_x_Cload[i]) * (_shared.data()->robot_response.state.F_x_Ctotal[j] - _shared.data()->robot_response.state.F_x_Cload[j])
    );
}

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
    if (!_command_running.compare_exchange_strong(expected, true)) throw ControlException("franka_emulator::Robot::automaticErrorRecovery: Command already running");

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