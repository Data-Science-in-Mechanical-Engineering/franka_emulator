#pragma once
#include <array>

namespace FRANKA_EMULATOR
{
    namespace emulator
    {
        static const double gripper_mass = 0.73;
        static const std::array<double, 9> gripper_inertia{{ 0.001, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0017 }};
        static const std::array<double, 16> gripper_frame{{ 0.7071, -0.7071, 0, 0, 0.7071, 0.7071, 0, 0, 0, 0, 1, 0, 0, 0, 0.1034, 1 }};
        static const double gripper_maximum_width = 0.08;
        static const double gripper_minimum_speed = 0.001;
        static const double gripper_maximum_speed = 20.0;
        static const double gripper_homing_speed = 0.1;
        static const double gripper_minimum_force = 0.1;
        static const double gripper_maximum_force = 20.0;
        static const double gripper_control_stiffness = 1.0;
        static const double gripper_control_damping = 2.0;
        static const double gripper_control_mismatch_treshold = 0.01;
        static const double gripper_control_move_mismatch = 0.001;

        static const std::array<double, 16> stiffness_frame{{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }};
    }
}