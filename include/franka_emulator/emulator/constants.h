#pragma once
#include <array>

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        //Gripper control
        static const double gripper_maximum_width = 0.08;
        static const double gripper_minimum_speed = 0.001;
        static const double gripper_maximum_speed = 20.0;
        static const double gripper_homing_speed = 0.1;
        static const double gripper_minimum_force = 0.1;
        static const double gripper_maximum_force = 20.0;
        static const double gripper_control_stiffness = 10.0;
        static const double gripper_control_damping = 6.0;
        static const double gripper_control_mismatch_treshold = 0.01;
        static const double gripper_control_move_mismatch = 0.001;
        static const unsigned int gripper_temperature = 25;
        
        //Gripepr dynamics
        static const double gripper_mass = 0.73;
        static const std::array<double, 9> gripper_inertia {{ 0.001, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017 }};
        static const std::array<double, 16> nominal_gripper_frame {{ 0.7071, -0.7071, 0, 0, 0.7071, 0.7071, 0, 0, 0, 0, 1, 0, 0, 0, 0.1034, 1 }};

        //Robot control
        static const double default_translational_stiffness = 100.0;
        static const double default_rotational_stiffness = 10.0;
        static const std::array<double, 7> default_joint_stiffness {{ 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0 }};
        static const std::array<double, 7> default_joint_damping {{ 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0 }};
    }
}