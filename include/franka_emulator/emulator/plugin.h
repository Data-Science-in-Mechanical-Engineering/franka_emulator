#include "shared.h"
#include "semaphore.h"
#include "network.h"
#include "../model.h"
#include <gazebo/gazebo.hh>
#include <string>
#include <pthread.h>

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        class Plugin : public gazebo::ModelPlugin
        {
        private:
            static const size_t _nanosecond_timeout = 300 * 1000;

            double _previous_torque[7] = { 0, 0, 0, 0, 0, 0, 0 };
            Network _franka_network;
            Model _franka_model;
            gazebo::physics::ModelPtr _model;
            gazebo::physics::JointPtr _joints[7];
            gazebo::physics::JointPtr _fingers[2];
            gazebo::event::ConnectionPtr _connection;
            Shared _shared;
            Semaphore _plugin_to_robot_mutex;
            Semaphore _plugin_to_robot_condition;
            Semaphore _robot_to_plugin_mutex;
            Semaphore _robot_to_plugin_condition;

        public:
            Plugin();
            virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual ~Plugin();
        };

        GZ_REGISTER_MODEL_PLUGIN(FRANKA_EMULATOR::emulator::Plugin)
    }
}