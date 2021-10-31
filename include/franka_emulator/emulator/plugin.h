#include "shared.h"
#include "semaphore.h"
#include "thread.h"
#include <gazebo/gazebo.hh>
#include <string>
#include <pthread.h>

namespace FRANKA_EMULATOR_CXX_NAME
{
    namespace emulator
    {
        class Plugin : public gazebo::ModelPlugin
        {
        private:
            static const size_t _nanosecond_timeout = 300 * 1000;

            bool _finish = false;
            gazebo::physics::ModelPtr _model;
            gazebo::physics::JointPtr _joints[7];
            gazebo::physics::JointPtr _fingers[2];

            Shared _shared;
            Semaphore _plugin_to_robot_mutex;
            Semaphore _plugin_to_robot_condition;
            Semaphore _robot_to_plugin_mutex;
            Semaphore _robot_to_plugin_condition;
            Thread _thread;

        public:
            Plugin();
            virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual ~Plugin();
        };

        GZ_REGISTER_MODEL_PLUGIN(FRANKA_EMULATOR_CXX_NAME::emulator::Plugin)
    }
}