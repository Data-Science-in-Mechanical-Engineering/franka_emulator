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
            std::string _shared_name    = "";
            int _shared_file            = -1;
            emulator::Shared *_shared   = nullptr;
            pthread_t _real_time_thread = -1;
            bool _finish                = false;

            gazebo::physics::ModelPtr _model;
            gazebo::physics::JointPtr _joints[7];
            gazebo::physics::JointPtr _fingers[2];

        public:
            Plugin();
            void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
            ~Plugin();
        };

        GZ_REGISTER_MODEL_PLUGIN(franka::emulator::Plugin)
    }
}