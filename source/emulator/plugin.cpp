#include "../../include/franka_emulator/emulator/plugin.h"
#include "../../include/franka_emulator/emulator/constants.h"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <sys/mman.h>
#include <sys/stat.h>
#include <pthread.h>
#include <fcntl.h>
#include <sched.h>
#include <unistd.h>
#include <iostream>
#include <algorithm>
#include <stdexcept>

FRANKA_EMULATOR::emulator::Plugin::Plugin() : _franka_model(_franka_network)
{
}

void FRANKA_EMULATOR::emulator::Plugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    try
    {
        //Initializing joints
        _model = model;
        for (size_t i = 0; i < 7; i++) _joints[i] = model->GetJoint(std::string("panda::panda_joint") + std::to_string(i + 1));
        for (size_t i = 0; i < 2; i++) _fingers[i] = model->GetJoint(std::string("panda::panda_finger_joint") + std::to_string(i + 1));

        //Initializing position
        _joints[0]->SetPosition(0, 0.0);
        _joints[1]->SetPosition(0, -M_PI / 4);
        _joints[2]->SetPosition(0, 0.0);
        _joints[3]->SetPosition(0, -3 * M_PI / 4);
        _joints[4]->SetPosition(0, 0.0);
        _joints[5]->SetPosition(0, M_PI / 2);
        _joints[6]->SetPosition(0, M_PI / 4);
        _fingers[0]->SetPosition(0, 0.0);
        _fingers[1]->SetPosition(0, 0.0);

        //Getting IP
        const char *cip = getenv("FRANKA_EMULATOR_IP");
        if (cip == nullptr) throw std::runtime_error("franka_emulator::emulator::Plugin::Load: FRANKA_EMULATOR_IP is not set");
        std::string ip = cip;

        //Opening shared memory
        _shared = Shared("/franka_emulator_" + ip + "_memory", true);
        
        //Opening semaphores
        _gripper_request_mutex = Semaphore("/franka_emulator_" + ip + "_gripper_request_mutex", true, 1);
        _gripper_response_mutex = Semaphore("/franka_emulator_" + ip + "_gripper_response_mutex", true, 1);
        _gripper_request_condition = Semaphore("/franka_emulator_" + ip + "_gripper_request_condition", true, 0);
        _gripper_atomic_response_condition = Semaphore("/franka_emulator_" + ip + "_gripper_atomic_response_condition", true, 0);
        _gripper_continuous_response_condition = Semaphore("/franka_emulator_" + ip + "_gripper_continuous_response_condition", true, 0);
        _robot_request_mutex = Semaphore("/franka_emulator_" + ip + "_robot_request_mutex", true, 1);
        _robot_response_mutex = Semaphore("/franka_emulator_" + ip + "_robot_response_mutex", true, 1);
        _robot_request_condition = Semaphore("/franka_emulator_" + ip + "_robot_request_condition", true, 0);
        _robot_response_condition = Semaphore("/franka_emulator_" + ip + "_robot_response_condition", true, 0);
        _robot_control_call_condition = Semaphore("/franka_emulator_" + ip + "_robot_control_call_condition", true, 0);
        _robot_control_return_condition = Semaphore("/franka_emulator_" + ip + "_robot_control_return_condition", true, 0);
        
        class Subscriber
        {
        private:
            Plugin *_plugin;
        public:
            Subscriber(Plugin *plugin) : _plugin(plugin)
            {
            }

            void operator()(const gazebo::common::UpdateInfo &info)
            {
                _plugin->_update(info);
            }
        } subscriber(this);
        _connection = gazebo::event::Events::ConnectWorldUpdateBegin(subscriber);
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return;
    }
    std::cerr << "franka_emulator::emulator::Plugin loaded" << std::endl;
}

void FRANKA_EMULATOR::emulator::Plugin::_update(const gazebo::common::UpdateInfo &info)
{
    std::lock_guard<std::mutex>(_model->GetWorld()->WorldPoseMutex());

    //Processing robot requests
    if (_robot_request_condition.value() != 0)
    {
        _robot_request_condition.wait();
        _robot_request_mutex.limitedpost(1);
        if (_shared.data()->robot_request.typ == RobotRequest::Type::read)
        {
            _robot_response_mutex.wait();
            for (size_t i = 0; i < 7; i++)
            {
                _shared.data()->robot_response.state.q[i] = _joints[i]->Position(0);
                _shared.data()->robot_response.state.dq[i] = _joints[i]->GetVelocity(0);
            }
            _shared.data()->robot_response.typ = RobotResponse::Type::ok;
            _robot_response_condition.limitedpost(1);
        }
        else if (_shared.data()->robot_request.typ == RobotRequest::Type::recovery)
        {
            _joints[0]->SetPosition(0, 0.0);
            _joints[1]->SetPosition(0, -M_PI / 4);
            _joints[2]->SetPosition(0, 0.0);
            _joints[3]->SetPosition(0, -3 * M_PI / 4);
            _joints[4]->SetPosition(0, 0.0);
            _joints[5]->SetPosition(0, M_PI / 2);
            _joints[6]->SetPosition(0, M_PI / 4);
            _fingers[0]->SetPosition(0, 0.0);
            _fingers[1]->SetPosition(0, 0.0);
            _robot_response_mutex.wait();
            _shared.data()->robot_response.typ = RobotResponse::Type::ok;
            _robot_response_condition.limitedpost(1);
        }
        else
        {
            _robot_state = RobotState::controlling;
            _robot_response_mutex.wait();
            _shared.data()->robot_response.typ = RobotResponse::Type::ok;
            _robot_response_condition.limitedpost(1);
        }
    }

    //Processing gripper requests
    if (_gripper_request_condition.value() != 0)
    {
        _gripper_request_condition.wait();
        if (_shared.data()->gripper_request.typ == GripperRequest::Type::read)
        {
            _gripper_request_mutex.limitedpost(1);

            _gripper_response_mutex.wait();
            _shared.data()->gripper_response.state.width = _fingers[0]->Position(0) + _fingers[1]->Position(0);
            _shared.data()->gripper_response.typ = GripperResponse::Type::ok;
            _gripper_atomic_response_condition.limitedpost(1);
        }
        else if (_shared.data()->gripper_request.typ == GripperRequest::Type::homing)
        {
            _gripper_request_mutex.limitedpost(1);
            
            _gripper_state = GripperState::homing;
            _gripper_target_speed = gripper_homing_speed;
            _gripper_current_target_width = _fingers[0]->Position(0) + _fingers[1]->Position(0);
            _gripper_previous_width = std::numeric_limits<double>::quiet_NaN();
            _gripper_opening = true;
        }
        else if (_shared.data()->gripper_request.typ == GripperRequest::Type::move)
        {
            _gripper_target_width = std::min(std::max(_shared.data()->gripper_request.width, 0.0), gripper_maximum_width);
            _gripper_target_speed = std::min(std::max(abs(_shared.data()->gripper_request.speed), gripper_minimum_speed), gripper_maximum_speed);
            _gripper_request_mutex.limitedpost(1);
            
            _gripper_state = GripperState::moving;
            _gripper_current_target_width = _fingers[0]->Position(0) + _fingers[1]->Position(0);
            _gripper_previous_width = std::numeric_limits<double>::quiet_NaN();
            _gripper_opening = _gripper_target_width > (_fingers[0]->Position(0) + _fingers[1]->Position(0));
            if (!_gripper_opening) _gripper_target_speed = -_gripper_target_speed;
        }
        else if (_shared.data()->gripper_request.typ == GripperRequest::Type::grasp)
        {
            _gripper_target_width = std::min(std::max(_shared.data()->gripper_request.width, 0.0), gripper_maximum_width);
            _gripper_target_speed = std::min(std::max(abs(_shared.data()->gripper_request.speed), gripper_minimum_speed), gripper_maximum_speed);
            _gripper_force = std::min(std::max(abs(_shared.data()->gripper_request.force), gripper_minimum_force), gripper_maximum_force);
            _gripper_epsilon_inner = std::max(_shared.data()->gripper_request.epsilon_inner, 0.0);
            _gripper_epsilon_outer = std::max(_shared.data()->gripper_request.epsilon_outer, 0.0);
            _gripper_request_mutex.limitedpost(1);
            
            _gripper_state = GripperState::grasping;
            _gripper_current_target_width = _fingers[0]->Position(0) + _fingers[1]->Position(0);
            _gripper_previous_width = std::numeric_limits<double>::quiet_NaN();
            _gripper_opening = _gripper_target_width > (_fingers[0]->Position(0) + _fingers[1]->Position(0));
            if (!_gripper_opening) _gripper_target_speed = -_gripper_target_speed;
        }
        else
        {
            _gripper_request_mutex.limitedpost(1);
            
            if (_gripper_state == GripperState::moving || _gripper_state == GripperState::grasping || _gripper_state == GripperState::homing)
            {
                _gripper_response_mutex.wait();
                _shared.data()->gripper_response.typ = GripperResponse::Type::error;
                _gripper_continuous_response_condition.limitedpost(1);
            }
            _gripper_response_mutex.wait();
            _shared.data()->gripper_response.typ = GripperResponse::Type::ok;
            _gripper_atomic_response_condition.limitedpost(1);
            _gripper_state = GripperState::idle;
        }
    }

    //Controlling robot
    for (size_t i = 0; i < 7; i++)
    {
        _shared.data()->robot_response.state.q[i] = _joints[i]->Position(0);
        _shared.data()->robot_response.state.dq[i] = _joints[i]->GetVelocity(0);
    }
    std::array<double, 7> gravity = _franka_model.gravity(_shared.data()->robot_response.state.q, gripper_mass, std::array<double, 3>({0,0,0}));
    for (size_t i = 0; i < 7; i++) _joints[i]->SetForce(0, gravity[i]);
    
    if (_robot_state == RobotState::controlling)
    {    
        _robot_control_call_condition.limitedpost(1);
        _robot_control_return_condition.wait();
        if (_shared.data()->robot_response.control_finished)
        {
            _shared.data()->robot_response.control_finished = false;
            _robot_state == RobotState::idle;
        }
        else
        {
            for (size_t i = 0; i < 7; i++) _joints[i]->SetForce(0, _shared.data()->robot_response.state.tau_J[i]);
        }
    }

    //Controlling gripper
    if (_gripper_state != GripperState::idle)
    {
        double width = _fingers[0]->Position(0) + _fingers[1]->Position(0);
        double force; //Gazebo becames instable on high forces. So I am using small forces if width is near current target and high forces if it is not
        if (abs(width - _gripper_current_target_width) > gripper_control_mismatch_treshold) force = _gripper_opening ? gripper_maximum_force : -gripper_maximum_force;
        else force = gripper_control_stiffness * (_gripper_current_target_width - width) + gripper_control_damping * (_gripper_target_speed - _fingers[0]->GetVelocity(0) - _fingers[1]->GetVelocity(0));

        if (_gripper_state == GripperState::moving)
        {
            if (((_gripper_opening && width >= _gripper_target_width) || (_gripper_opening && _gripper_previous_width <= width && width + gripper_control_move_mismatch >= _gripper_target_width))
            || ((!_gripper_opening && width <= _gripper_target_width) || (!_gripper_opening && _gripper_previous_width >= width && width - gripper_control_move_mismatch <= _gripper_target_width)))
            {
                _fingers[0]->SetForce(0, 0.0);
                _fingers[1]->SetForce(0, 0.0);
                _gripper_state = GripperState::idle;
                _gripper_response_mutex.wait();
                _shared.data()->gripper_response.typ = GripperResponse::Type::ok;
                _gripper_continuous_response_condition.limitedpost(1);
            }
            else
            {
                _fingers[0]->SetForce(0, force);
                _fingers[1]->SetForce(0, force);
            }
        }
        else if (_gripper_state == GripperState::grasping)
        {
            force = std::max(std::min(force, _gripper_force), -_gripper_force);
            if ((_gripper_opening && _gripper_previous_width >= width) || (!_gripper_opening && _gripper_previous_width <= width))
            {
                _fingers[0]->SetForce(0,  0.0);
                _fingers[1]->SetForce(0,  0.0);
                _gripper_state = GripperState::idle;
                _gripper_response_mutex.wait();
                _shared.data()->gripper_response.typ = ((width >= _gripper_target_width - _gripper_epsilon_inner) && (width <= _gripper_target_width + _gripper_epsilon_outer)) ? GripperResponse::Type::ok : GripperResponse::Type::error;
                _gripper_continuous_response_condition.limitedpost(1);
            }
            else
            {
                _fingers[0]->SetForce(0, force);
                _fingers[1]->SetForce(0, force);
            }
        }
        else if (_gripper_state == GripperState::homing)
        {
            if (_gripper_opening && _gripper_previous_width >= width)
            {
                _gripper_maximum_width = _gripper_previous_width;
                _gripper_opening = false;
                _gripper_target_speed = -_gripper_target_speed;
            }
            else if (!_gripper_opening && _gripper_previous_width <= width)
            {
                _fingers[0]->SetForce(0,  0.0);
                _fingers[1]->SetForce(0,  0.0);
                _gripper_state = GripperState::idle;
                _gripper_response_mutex.wait();
                _shared.data()->gripper_response.typ = GripperResponse::Type::ok;
                _gripper_continuous_response_condition.limitedpost(1);
            }
            else
            {
                _fingers[0]->SetForce(0,  force);
                _fingers[1]->SetForce(0,  force);
            }
        }
        _gripper_previous_width = width;
        _gripper_current_target_width = std::min(std::max(_gripper_current_target_width + 0.001 * _gripper_target_speed, 0.0), gripper_maximum_width);
    }
    else
    {
        double width = _fingers[0]->Position(0) + _fingers[1]->Position(0);
        double force = gripper_control_stiffness * (_gripper_previous_width - width) + gripper_control_damping * (- _fingers[0]->GetVelocity(0) - _fingers[1]->GetVelocity(0));
        _fingers[0]->SetForce(0,  force);
        _fingers[1]->SetForce(0,  force);   
    }
}

FRANKA_EMULATOR::emulator::Plugin::~Plugin()
{
    std::cerr << "franka_emulator::emulator::Plugin unloaded" << std::endl;
}