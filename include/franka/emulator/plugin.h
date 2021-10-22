#include "shared.h"
#include <gazebo/gazebo.hh>
#include <string>
#include <pthread.h>

namespace franka
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

            std::string _ip                     = "";
            int _shared_file                    = -1;
            sem_t *_plugin_to_robot_mutex     = nullptr;
            sem_t *_plugin_to_robot_condition = nullptr;
            sem_t *_robot_to_plugin_mutex     = nullptr;
            sem_t *_robot_to_plugin_condition = nullptr;
            emulator::Shared *_shared           = nullptr;
            pthread_t _real_time_thread         = -1;

        public:
            Plugin();
            virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual ~Plugin();
        };

        GZ_REGISTER_MODEL_PLUGIN(franka::emulator::Plugin)
    }
}