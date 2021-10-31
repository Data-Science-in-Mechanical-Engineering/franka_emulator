#include "../../include/franka_emulator/emulator/plugin.h"
#include "../../include/franka_emulator/emulator/network.h"
#include "../../include/franka_emulator/model.h"
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

FRANKA_EMULATOR_CXX_NAME::emulator::Plugin::Plugin()
{
}

void FRANKA_EMULATOR_CXX_NAME::emulator::Plugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
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
        
        //Creating real-time thread
        _thread = Thread(90, this, [](void* uncasted_plugin) -> void*
        {
            Plugin *plugin = (Plugin*)uncasted_plugin;
            FRANKA_EMULATOR_CXX_NAME::Network dummy;
            FRANKA_EMULATOR_CXX_NAME::Model model(dummy);
            double previous_torque[7] = { 0, 0, 0, 0, 0, 0, 0 };
            struct timespec time;
            clock_gettime(CLOCK_MONOTONIC, &time);
            while (!plugin->_finish)
            {
                time.tv_nsec += 1000*1000;
                if (time.tv_nsec > 1000*1000*1000) { time.tv_nsec -= 1000*1000*1000; time.tv_sec++; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, nullptr);
                if (plugin->_model->GetWorld()->IsPaused()) continue;
                plugin->_model->GetWorld()->WorldPoseMutex().lock();
                
                plugin->_plugin_to_robot_mutex.wait();
                for (size_t i = 0; i < 7; i++)
                {
                    plugin->_shared.data()->robot_state.q[i] = plugin->_joints[i]->Position(0);
                    plugin->_shared.data()->robot_state.dq[i] = plugin->_joints[i]->GetVelocity(0);
                }
                plugin->_plugin_to_robot_condition.limitedpost(1);
                plugin->_plugin_to_robot_mutex.post();

                
                plugin->_robot_to_plugin_condition.timedwait(_nanosecond_timeout);
                plugin->_robot_to_plugin_mutex.wait();
                
                std::array<double, 7> current_torque = model.gravity(plugin->_shared.data()->robot_state);
                for (size_t i = 0; i < 7; i++)
                {
                    current_torque[i] += plugin->_shared.data()->robot_state.tau_J[i];
                    plugin->_joints[i]->SetForce(0, -previous_torque[i]);
                    plugin->_joints[i]->SetForce(0, current_torque[i]);
                    previous_torque[i] = current_torque[i];
                }
                plugin->_robot_to_plugin_mutex.post();
                
                plugin->_model->GetWorld()->WorldPoseMutex().unlock();
            }
            return nullptr;
        });
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return;
    }
    std::cerr << "franka_emulator::emulator::Plugin loaded" << std::endl;
}

FRANKA_EMULATOR_CXX_NAME::emulator::Plugin::~Plugin()
{
    _finish = true;
    _thread.join();
    std::cerr << "franka_emulator::emulator::Plugin unloaded" << std::endl;
}