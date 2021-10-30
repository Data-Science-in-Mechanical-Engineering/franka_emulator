#include "../include/franka_emulator/errors.h"

FRANKA_EMULATOR_CXX_NAME::Errors::Errors() : Errors(std::array<bool, 37>{})
{}

FRANKA_EMULATOR_CXX_NAME::Errors::Errors(const Errors& other) : Errors(other.errors_)
{
    errors_ = other.errors_;
}

FRANKA_EMULATOR_CXX_NAME::Errors& FRANKA_EMULATOR_CXX_NAME::Errors::operator=(Errors other)
{
    errors_ = other.errors_;
    return *this;
}

FRANKA_EMULATOR_CXX_NAME::Errors::Errors(const std::array<bool, 37>& errors) :
    errors_(errors),
    joint_position_limits_violation(errors_[0]),
    cartesian_position_limits_violation(errors_[1]),
    self_collision_avoidance_violation(errors_[2]),
    joint_velocity_violation(errors_[3]),
    cartesian_velocity_violation(errors_[4]),
    force_control_safety_violation(errors_[5]),
    joint_reflex(errors_[6]),
    cartesian_reflex(errors_[7]),
    max_goal_pose_deviation_violation(errors_[8]),
    max_path_pose_deviation_violation(errors_[9]),
    cartesian_velocity_profile_safety_violation(errors_[10]),
    joint_position_motion_generator_start_pose_invalid(errors_[11]),
    joint_motion_generator_position_limits_violation(errors_[12]),
    joint_motion_generator_velocity_limits_violation(errors_[13]),
    joint_motion_generator_velocity_discontinuity(errors_[14]),
    joint_motion_generator_acceleration_discontinuity(errors_[15]),
    cartesian_position_motion_generator_start_pose_invalid(errors_[16]),
    cartesian_motion_generator_elbow_limit_violation(errors_[17]),
    cartesian_motion_generator_velocity_limits_violation(errors_[18]),
    cartesian_motion_generator_velocity_discontinuity(errors_[19]),
    cartesian_motion_generator_acceleration_discontinuity(errors_[20]),
    cartesian_motion_generator_elbow_sign_inconsistent(errors_[21]),
    cartesian_motion_generator_start_elbow_invalid(errors_[22]),
    cartesian_motion_generator_joint_position_limits_violation(errors_[23]),
    cartesian_motion_generator_joint_velocity_limits_violation(errors_[24]),
    cartesian_motion_generator_joint_velocity_discontinuity(errors_[25]),
    cartesian_motion_generator_joint_acceleration_discontinuity(errors_[26]),
    cartesian_position_motion_generator_invalid_frame(errors_[27]),
    force_controller_desired_force_tolerance_violation(errors_[28]),
    controller_torque_discontinuity(errors_[29]),
    start_elbow_sign_inconsistent(errors_[30]),
    communication_constraints_violation(errors_[31]),
    power_limit_violation(errors_[32]),
    joint_p2p_insufficient_torque_for_planning(errors_[33]),
    tau_j_range_violation(errors_[34]),
    instability_detected(errors_[35]),
    joint_move_in_wrong_direction(errors_[36])
{
    errors_ = errors;
}

FRANKA_EMULATOR_CXX_NAME::Errors::operator bool() const noexcept
{
    for (size_t i = 0; i < 37; i++)
    {
        if (errors_[i]) return true;
    }
    return false;
}

FRANKA_EMULATOR_CXX_NAME::Errors::operator std::string() const
{
    std::string result = "[";
    if (joint_position_limits_violation) result += "joint_position_limits_violation, ";
    if (cartesian_position_limits_violation) result += "cartesian_position_limits_violation, ";
    if (self_collision_avoidance_violation) result += "self_collision_avoidance_violation, ";
    if (joint_velocity_violation) result += "joint_velocity_violation, ";
    if (cartesian_velocity_violation) result += "cartesian_velocity_violation, ";
    if (force_control_safety_violation) result += "force_control_safety_violation, ";
    if (joint_reflex) result += "joint_reflex, ";
    if (cartesian_reflex) result += "cartesian_reflex, ";
    if (max_goal_pose_deviation_violation) result += "max_goal_pose_deviation_violation, ";
    if (max_path_pose_deviation_violation) result += "max_path_pose_deviation_violation, ";
    if (cartesian_velocity_profile_safety_violation) result += "cartesian_velocity_profile_safety_violation, ";
    if (joint_position_motion_generator_start_pose_invalid) result += "joint_position_motion_generator_start_pose_invalid, ";
    if (joint_motion_generator_position_limits_violation) result += "joint_motion_generator_position_limits_violation, ";
    if (joint_motion_generator_velocity_limits_violation) result += "joint_motion_generator_velocity_limits_violation, ";
    if (joint_motion_generator_velocity_discontinuity) result += "joint_motion_generator_velocity_discontinuity, ";
    if (joint_motion_generator_acceleration_discontinuity) result += "joint_motion_generator_acceleration_discontinuity, ";
    if (cartesian_position_motion_generator_start_pose_invalid) result += "cartesian_position_motion_generator_start_pose_invalid, ";
    if (cartesian_motion_generator_elbow_limit_violation) result += "cartesian_motion_generator_elbow_limit_violation, ";
    if (cartesian_motion_generator_velocity_limits_violation) result += "cartesian_motion_generator_velocity_limits_violation, ";
    if (cartesian_motion_generator_velocity_discontinuity) result += "cartesian_motion_generator_velocity_discontinuity, ";
    if (cartesian_motion_generator_acceleration_discontinuity) result += "cartesian_motion_generator_acceleration_discontinuity, ";
    if (cartesian_motion_generator_elbow_sign_inconsistent) result += "cartesian_motion_generator_elbow_sign_inconsistent, ";
    if (cartesian_motion_generator_start_elbow_invalid) result += "cartesian_motion_generator_start_elbow_invalid, ";
    if (cartesian_motion_generator_joint_position_limits_violation) result += "cartesian_motion_generator_joint_position_limits_violation, ";
    if (cartesian_motion_generator_joint_velocity_limits_violation) result += "cartesian_motion_generator_joint_velocity_limits_violation, ";
    if (cartesian_motion_generator_joint_velocity_discontinuity) result += "cartesian_motion_generator_joint_velocity_discontinuity, ";
    if (cartesian_motion_generator_joint_acceleration_discontinuity) result += "cartesian_motion_generator_joint_acceleration_discontinuity, ";
    if (cartesian_position_motion_generator_invalid_frame) result += "cartesian_position_motion_generator_invalid_frame, ";
    if (force_controller_desired_force_tolerance_violation) result += "force_controller_desired_force_tolerance_violation, ";
    if (controller_torque_discontinuity) result += "controller_torque_discontinuity, ";
    if (start_elbow_sign_inconsistent) result += "start_elbow_sign_inconsistent, ";
    if (communication_constraints_violation) result += "communication_constraints_violation, ";
    if (power_limit_violation) result += "power_limit_violation, ";
    if (joint_p2p_insufficient_torque_for_planning) result += "joint_p2p_insufficient_torque_for_planning, ";
    if (tau_j_range_violation) result += "tau_j_range_violation, ";
    if (instability_detected) result += "instability_detected, ";
    if (joint_move_in_wrong_direction) result += "joint_move_in_wrong_direction, ";
    if (*this) result.resize(result.size() - 2); //Delete last comma and space
    result += "]";
    return result;
}

std::ostream& operator<<(std::ostream& ostream, const FRANKA_EMULATOR_CXX_NAME::Errors& errors)
{
    ostream << static_cast<std::string>(errors);
    return ostream;
}