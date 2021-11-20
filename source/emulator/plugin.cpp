#include "../../include/franka_emulator/emulator/plugin.h"
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
        _plugin_to_robot_mutex = Semaphore("/franka_emulator_" + ip + "_plugin_to_robot_mutex", true, 1);
        _plugin_to_robot_condition = Semaphore("/franka_emulator_" + ip + "_plugin_to_robot_condition", true, 0);
        _robot_to_plugin_mutex = Semaphore("/franka_emulator_" + ip + "_robot_to_plugin_mutex", true, 1);
        _robot_to_plugin_condition = Semaphore("/franka_emulator_" + ip + "_robot_to_plugin_condition", true, 0);
        
        class Subscriber
        {
        private:
            Plugin *_plugin;
        public:
            Subscriber(Plugin *plugin) : _plugin(plugin)
            {
            }

            void operator()(const gazebo::common::UpdateInfo&)
            {
                _plugin->_model->GetWorld()->WorldPoseMutex().lock();
                
                //Sending data to robot
                _plugin->_plugin_to_robot_mutex.wait();
                for (size_t i = 0; i < 7; i++)
                {
                    _plugin->_shared.data()->robot_state.q[i] = _plugin->_joints[i]->Position(0);
                    _plugin->_shared.data()->robot_state.dq[i] = _plugin->_joints[i]->GetVelocity(0);
                }
                _plugin->_plugin_to_robot_condition.limitedpost(1);
                _plugin->_plugin_to_robot_mutex.post();
                
                //Receiving data from robot
                _plugin->_robot_to_plugin_condition.timedwait(_nanosecond_timeout);
                _plugin->_robot_to_plugin_mutex.wait();
                std::array<double, 7> gravity = _plugin->_franka_model.gravity(_plugin->_shared.data()->robot_state.q, 0.73, std::array<double, 3>({0,0,0}));
                for (size_t i = 0; i < 7; i++) _plugin->_joints[i]->SetForce(0, _plugin->_shared.data()->robot_state.tau_J[i] + gravity[i]);
                _plugin->_robot_to_plugin_mutex.post();
                
                _plugin->_model->GetWorld()->WorldPoseMutex().unlock();
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

FRANKA_EMULATOR::emulator::Plugin::~Plugin()
{
    std::cerr << "franka_emulator::emulator::Plugin unloaded" << std::endl;
}