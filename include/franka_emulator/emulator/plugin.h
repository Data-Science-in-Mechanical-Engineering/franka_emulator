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
            //Robot logic
            enum class RobotState
            {
                idle,
                controlling
            };
            RobotState _robot_state = RobotState::idle;

            //Gripper logic
            enum class GripperState
            {
                idle,
                moving,
                grasping,
                homing
            };
            GripperState _gripper_state = GripperState::idle;
            bool _gripper_opening = false;
            double _gripper_current_target_width = 0.0;
            double _gripper_previous_width = 0.0;
            double _gripper_maximum_width = 0.0;
            double _gripper_target_width = 0.0;
            double _gripper_target_speed = 0.0;
            double _gripper_force = 0.0;
            double _gripper_epsilon_inner = 0.0, _gripper_epsilon_outer = 0.0;
            
            //Physical model
            Network _franka_network;
            Model _franka_model;

            //Gazebo
            gazebo::physics::ModelPtr _model;
            gazebo::physics::JointPtr _joints[7];
            gazebo::physics::JointPtr _fingers[2];
            gazebo::event::ConnectionPtr _connection;
            void _update(const gazebo::common::UpdateInfo &info);

            //Interprocess
            Shared _shared;
            Semaphore _gripper_request_mutex;
            Semaphore _gripper_response_mutex;
            Semaphore _gripper_request_condition;
            Semaphore _gripper_atomic_response_condition;
            Semaphore _gripper_continuous_response_condition;
            Semaphore _robot_request_mutex;
            Semaphore _robot_response_mutex;
            Semaphore _robot_request_condition;
            Semaphore _robot_response_condition;
            Semaphore _robot_control_call_condition;
            Semaphore _robot_control_return_condition;

        public:
            Plugin();
            virtual void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual ~Plugin();
        };

        GZ_REGISTER_MODEL_PLUGIN(FRANKA_EMULATOR::emulator::Plugin)
    }
}