#include "../include/franka/emulator/plugin.h"
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

franka::emulator::Plugin::Plugin()
{
}

void franka::emulator::Plugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
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

    //Opening shared memory
    _shared_name = std::string("/franka_emulator_") + "192.168.0.1";
    _shared_file = shm_open(_shared_name.c_str(), O_CREAT | O_RDWR, 0644);
    if (_shared_file < 0) throw std::runtime_error("franka_emulator::Plugin::Load: shm_open failed");
    _shared = (emulator::Shared*) mmap(nullptr, sizeof(emulator::Shared), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, _shared_file, 0);
    if (_shared == MAP_FAILED) throw std::runtime_error("franka_emulator::Plugin::Load: mmap failed");

    //Creating mutexes
    struct MutexAttributes
    {
        pthread_mutexattr_t data;
        MutexAttributes() { if (pthread_mutexattr_init(&data) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_mutexattr_init failed"); }
        ~MutexAttributes() { pthread_mutexattr_destroy(&data); }
    } mutex_attributes;
    pthread_mutexattr_setpshared(&mutex_attributes.data, PTHREAD_PROCESS_SHARED);
    if(pthread_mutex_init(&_shared->robot_to_plugin_mutex, &mutex_attributes.data) != 0
    || pthread_mutex_init(&_shared->plugin_to_robot_mutex, &mutex_attributes.data) != 0) throw std::runtime_error("franka_timeout_handler: pthread_mutex_init failed");

    //Creating condition variables
    struct ConditionAttributes
    {
        pthread_condattr_t data;
        ConditionAttributes() { if (pthread_condattr_init(&data) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_condattr_init failed"); }
        ~ConditionAttributes() { pthread_condattr_destroy(&data); }
    } condition_attributes;
    pthread_condattr_setpshared(&condition_attributes.data, PTHREAD_PROCESS_SHARED);
    if(pthread_cond_init(&_shared->robot_to_plugin_condition, &condition_attributes.data) != 0
    || pthread_cond_init(&_shared->plugin_to_robot_condition, &condition_attributes.data) != 0) throw std::runtime_error("franka_timeout_handler: pthread_cond_init failed");

    //Creating real-time thread
    struct PthreadAttributes
    {
        pthread_attr_t data;
        PthreadAttributes() { if (pthread_attr_init(&data) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_attr_init failed"); }
        ~PthreadAttributes() { pthread_attr_destroy(&data); }
    } pthread_attributes;
    if (pthread_attr_setschedpolicy(&pthread_attributes.data, SCHED_FIFO) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_attr_setschedpolicy failed");
    sched_param scheduling_parameters;
    scheduling_parameters.sched_priority = 90;
    if (pthread_attr_setschedparam(&pthread_attributes.data, &scheduling_parameters) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_attr_setschedpolicy failed");
    if (pthread_attr_setinheritsched(&pthread_attributes.data, PTHREAD_EXPLICIT_SCHED) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_attr_setinheritsched failed");
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
            for (size_t i = 0; i < 7; i++)
            {
                plugin->_shared->robot_state.q[i] = plugin->_joints[i]->Position(0);
                plugin->_shared->robot_state.dq[i] = plugin->_joints[i]->GetVelocity(0);
            }
            
            pthread_mutex_lock(&plugin->_shared->plugin_to_robot_mutex);
            pthread_cond_signal(&plugin->_shared->plugin_to_robot_condition);
            pthread_mutex_unlock(&plugin->_shared->plugin_to_robot_mutex);

            struct timespec timeout;
            timeout.tv_sec = 0;
            timeout.tv_nsec = 300 * 1000;
            pthread_mutex_lock(&plugin->_shared->robot_to_plugin_mutex);
            pthread_cond_timedwait(&plugin->_shared->robot_to_plugin_condition, &plugin->_shared->robot_to_plugin_mutex, &timeout);
            pthread_mutex_unlock(&plugin->_shared->robot_to_plugin_mutex);
            
            for (size_t i = 0; i < 7; i++) plugin->_joints[i]->SetForce(0, plugin->_shared->robot_state.tau_J[i]);
            plugin->_model->GetWorld()->WorldPoseMutex().unlock();
        }
        return nullptr;
    }, this) != 0) throw std::runtime_error("franka_emulator::Plugin::Load: pthread_create failed");
    std::cerr << "franka::emulator::Plugin loaded" << std::endl;
}

franka::emulator::Plugin::~Plugin()
{
    if (_shared != nullptr && _shared != MAP_FAILED)
    {
        pthread_mutex_destroy(&_shared->robot_to_plugin_mutex);
        pthread_mutex_destroy(&_shared->plugin_to_robot_mutex);
        pthread_cond_destroy(&_shared->robot_to_plugin_condition);
        pthread_cond_destroy(&_shared->plugin_to_robot_condition);
        shm_unlink(_shared_name.c_str());
    }
    _finish = true;
    pthread_join(_real_time_thread, nullptr);
    std::cerr << "franka::emulator::Plugin unloaded" << std::endl;
}