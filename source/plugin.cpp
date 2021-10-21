#include "../include/franka/emulator/plugin.h"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdexcept>

franka::emulator::Plugin::Plugin()
{
}

void franka::emulator::Plugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
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
        const char *ip = getenv("FRANKA_EMULATOR_IP");
        if (ip == nullptr) throw std::runtime_error("franka::emulator::Plugin::Load: FRANKA_EMULATOR_IP is not set");
        _ip = ip;

        //Opening shared memory
        _shared_file = shm_open(("/franka_emulator_" + _ip + "_memory").c_str(), O_CREAT | O_TRUNC | O_RDWR, 0644);
        if (_shared_file < 0) throw std::runtime_error("franka::emulator::Robot::Robot: shm_open failed");
        _shared = (emulator::Shared*) mmap(nullptr, sizeof(emulator::Shared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, _shared_file, 0);
        if (_shared == MAP_FAILED) throw std::runtime_error("franka::emulator::Robot::Robot: mmap failed");
 
        //Opening semaphores
        _plugin_to_robot_mutex = sem_open(("/franka_emulator_" + _ip + "_plugin_to_robot_mutex").c_str(), O_CREAT, 0644, 1);
        if (_plugin_to_robot_mutex == SEM_FAILED) throw std::runtime_error("franka::emulator::Robot::Robot: sem_open failed");
        _plugin_to_robot_condition = sem_open(("/franka_emulator_" + _ip + "_plugin_to_robot_condition").c_str(), O_CREAT, 0644, 0);
        if (_plugin_to_robot_condition == SEM_FAILED) throw std::runtime_error("franka::emulator::Robot::Robot: sem_open failed");
        _robot_to_plugin_mutex = sem_open(("/franka_emulator_" + _ip + "_robot_to_plugin_mutex").c_str(), O_CREAT, 0644, 1);
        if (_robot_to_plugin_mutex == SEM_FAILED) throw std::runtime_error("franka::emulator::Robot::Robot: sem_open failed");
        _robot_to_plugin_condition = sem_open(("/franka_emulator_" + _ip + "_robot_to_plugin_condition").c_str(), O_CREAT, 0644, 0);
        if (_robot_to_plugin_condition == SEM_FAILED) throw std::runtime_error("franka::emulator::Robot::Robot: sem_open failed");
 
        //Creating real-time thread
        struct PthreadAttributes
        {
            pthread_attr_t data;
            PthreadAttributes() { if (pthread_attr_init(&data) != 0) throw std::runtime_error("franka::emulator::Plugin::Load: pthread_attr_init failed"); }
            ~PthreadAttributes() { pthread_attr_destroy(&data); }
        } pthread_attributes;
        if (pthread_attr_setschedpolicy(&pthread_attributes.data, SCHED_FIFO) != 0) throw std::runtime_error("franka::emulator::Plugin::Load: pthread_attr_setschedpolicy failed");
        sched_param scheduling_parameters;
        scheduling_parameters.sched_priority = 90;
        if (pthread_attr_setschedparam(&pthread_attributes.data, &scheduling_parameters) != 0) throw std::runtime_error("franka::emulator::Plugin::Load: pthread_attr_setschedpolicy failed");
        if (pthread_attr_setinheritsched(&pthread_attributes.data, PTHREAD_EXPLICIT_SCHED) != 0) throw std::runtime_error("franka::emulator::Plugin::Load: pthread_attr_setinheritsched failed");
        if (pthread_create(&_real_time_thread, &pthread_attributes.data, [](void* uncasted_plugin) -> void*
        {
            Plugin *plugin = (Plugin*)uncasted_plugin;
            struct timespec time;
            clock_gettime(CLOCK_MONOTONIC, &time);
            while (!plugin->_finish)
            {
                time.tv_nsec += 1000*1000;
                if (time.tv_nsec > 1000*1000*1000) { time.tv_nsec -= 1000*1000*1000; time.tv_sec++; }
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time, nullptr);
                plugin->_model->GetWorld()->WorldPoseMutex().lock();
                
                std::cerr << "Signaling" << std::endl;
                sem_wait(plugin->_plugin_to_robot_mutex);
                for (size_t i = 0; i < 7; i++)
                {
                    plugin->_shared->robot_state.q[i] = plugin->_joints[i]->Position(0);
                    plugin->_shared->robot_state.dq[i] = plugin->_joints[i]->GetVelocity(0);
                }
                int plugin_to_robot_condition;
                sem_getvalue(plugin->_plugin_to_robot_condition, &plugin_to_robot_condition);
                if (plugin_to_robot_condition == 0) sem_post(plugin->_plugin_to_robot_condition);
                sem_post(plugin->_plugin_to_robot_mutex);

                std::cerr << "Waiting" << std::endl;
                timespec timeout;
                clock_gettime(CLOCK_REALTIME, &timeout);
                timeout.tv_nsec += _nanosecond_timeout;
                if (timeout.tv_nsec > 1000*1000*1000) { timeout.tv_nsec -= 1000*1000*1000; timeout.tv_sec++; }
                sem_timedwait(plugin->_robot_to_plugin_condition, &timeout);
                sem_wait(plugin->_robot_to_plugin_mutex);
                for (size_t i = 0; i < 7; i++)
                {
                    plugin->_joints[i]->SetForce(0, plugin->_shared->robot_state.tau_J[i]);
                }
                sem_post(plugin->_robot_to_plugin_mutex);
                
                plugin->_model->GetWorld()->WorldPoseMutex().unlock();
            }
            return nullptr;
        }, this) != 0) throw std::runtime_error("franka::emulator::Plugin::Load: pthread_create failed");
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return;
    }
    std::cerr << "franka::emulator::Plugin loaded" << std::endl;
}

franka::emulator::Plugin::~Plugin()
{
    _finish = true;
    pthread_join(_real_time_thread, nullptr);

    if (_shared != nullptr && _shared != MAP_FAILED)
    {
        munmap(_shared, sizeof(emulator::Shared));
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

    std::cerr << "franka::emulator::Plugin unloaded" << std::endl;
}