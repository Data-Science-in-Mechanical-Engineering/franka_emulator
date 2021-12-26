#include "shared.h"
#include "semaphore.h"
#include "network.h"
#include "constants.h"
#include "../model.h"
#include <gazebo/gazebo.hh>
#include <string>
#include <memory>
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
            double _robot_previous_torque[7] = {0,0,0,0,0,0,0};

            //Gripper logic
            enum class GripperState
            {
                idle,
                moving,
                grasping,
                homing
            };
            GripperState _gripper_state     = GripperState::idle;   //Gripper command
            bool _gripper_opening           = false;                //Gripper command direction
            double _gripper_width           = 0.0;                  //Width argument
            double _gripper_speed           = 0.0;                  //Speed argument
            double _gripper_force           = 0.0;                  //Force argument
            double _gripper_epsilon_inner   = 0.0;                  //Inner epsilon argument
            double _gripper_epsilon_outer   = 0.0;                  //Outer epsilon argument
            bool _gripper_grasped           = false;                //Last result of grasp() and indicator if needs to apply force when idle
            double _gripper_maximum_width   = 0.0;                  //Last result of homing()
            double _gripper_current_width   = 0.0;                  //Current target width
            size_t _gripper_history_size    = 0;                    //Counter how many of history is valid
            size_t _gripper_history_pointer = 0;                    //Pointer to previous width
            double _gripper_history[gripper_history_size];          //Previous widths (used for stuck detection. Yes, can be implemented more efficiently, but I keep things explicit)
            
            //Physical model
            std::unique_ptr<Model> _physical_model;

            //Gazebo
            gazebo::physics::ModelPtr _model;
            gazebo::physics::JointPtr _joints[7];
            gazebo::physics::JointPtr _fingers[2];
            gazebo::event::ConnectionPtr _connection;
            void _set_default_position();
            void _init_robot_state();
            void _fill_robot_state(const gazebo::common::Time &time);
            void _init_gripper_state();
            void _fill_gripper_state(const gazebo::common::Time &time);
            void _process_robot_request(const gazebo::common::Time &time);
            void _process_gripper_request(const gazebo::common::Time &time);
            void _control_robot(const gazebo::common::Time &time);
            void _control_gripper();
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