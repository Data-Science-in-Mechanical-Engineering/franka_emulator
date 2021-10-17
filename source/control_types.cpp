#include "../include/franka/control_types.h"

franka::Torques::Torques(const std::array<double, 7>& torques) noexcept : tau_J(torques)
{}

franka::Torques::Torques(std::initializer_list<double> torques)
{
    for (size_t i = 0; i < 7; i++) tau_J[i] = 0.0;
    std::initializer_list<double>::const_iterator c = torques.begin();
    for (size_t i = 0; i < torques.size(); i++, c++) tau_J[i] = *c;
}

franka::JointPositions::JointPositions(const std::array<double, 7>& joint_positions) noexcept : q(joint_positions)
{}

franka::JointPositions::JointPositions(std::initializer_list<double> joint_positions)
{
    for (size_t i = 0; i < 7; i++) q[i] = 0.0;
    std::initializer_list<double>::const_iterator c = joint_positions.begin();
    for (size_t i = 0; i < joint_positions.size(); i++, c++) q[i] = *c;
}

franka::JointVelocities::JointVelocities(const std::array<double, 7>& joint_velocities) noexcept : dq(joint_velocities)
{}

franka::JointVelocities::JointVelocities(std::initializer_list<double> joint_velocities)
{
    for (size_t i = 0; i < 7; i++) dq[i] = 0.0;
    std::initializer_list<double>::const_iterator c = joint_velocities.begin();
    for (size_t i = 0; i < joint_velocities.size(); i++, c++) dq[i] = *c;
}

franka::CartesianPose::CartesianPose(const std::array<double, 16>& cartesian_pose) noexcept : O_T_EE(cartesian_pose), elbow{0.0, 0.0}
{
}

franka::CartesianPose::CartesianPose(const std::array<double, 16>& cartesian_pose, const std::array<double, 2>& elbow) noexcept : O_T_EE(cartesian_pose), elbow(elbow)
{
}

franka::CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose) : elbow{0.0, 0.0}
{
    for (size_t i = 0; i < 16; i++) O_T_EE[i] = 0.0;
    std::initializer_list<double>::const_iterator c = cartesian_pose.begin();
    for (size_t i = 0; i < cartesian_pose.size(); i++, c++) O_T_EE[i] = *c;
}

franka::CartesianPose::CartesianPose(std::initializer_list<double> cartesian_pose, std::initializer_list<double> elbow)
{
    for (size_t i = 0; i < 16; i++) O_T_EE[i] = 0.0;
    std::initializer_list<double>::const_iterator c = cartesian_pose.begin();
    for (size_t i = 0; i < cartesian_pose.size(); i++, c++) O_T_EE[i] = *c;

    for (size_t i = 0; i < 2; i++) this->elbow[i] = 0.0;
    std::initializer_list<double>::const_iterator c2 = elbow.begin();
    for (size_t i = 0; i < elbow.size(); i++, c2++) this->elbow[i] = *c;
}

bool franka::CartesianPose::hasElbow() const noexcept
{
    return false;
}

franka::CartesianVelocities::CartesianVelocities(const std::array<double, 6>& cartesian_velocities) noexcept : O_dP_EE(cartesian_velocities), elbow{0.0, 0.0}
{
}

franka::CartesianVelocities::CartesianVelocities(const std::array<double, 6>& cartesian_velocities, const std::array<double, 2>& elbow) noexcept : O_dP_EE(cartesian_velocities), elbow(elbow)
{}

franka::CartesianVelocities::CartesianVelocities(std::initializer_list<double> cartesian_velocities)
{
    for (size_t i = 0; i < 6; i++) O_dP_EE[i] = 0.0;
    std::initializer_list<double>::const_iterator c = cartesian_velocities.begin();
    for (size_t i = 0; i < cartesian_velocities.size(); i++, c++) O_dP_EE[i] = *c;
}

franka::CartesianVelocities::CartesianVelocities(std::initializer_list<double> cartesian_velocities, std::initializer_list<double> elbow)
{
    for (size_t i = 0; i < 6; i++) O_dP_EE[i] = 0.0;
    std::initializer_list<double>::const_iterator c = cartesian_velocities.begin();
    for (size_t i = 0; i < cartesian_velocities.size(); i++, c++) O_dP_EE[i] = *c;

    for (size_t i = 0; i < 2; i++) this->elbow[i] = 0.0;
    std::initializer_list<double>::const_iterator c2 = elbow.begin();
    for (size_t i = 0; i < elbow.size(); i++, c2++) this->elbow[i] = *c;
}

  
bool franka::CartesianVelocities::hasElbow() const noexcept
{
  return false;
}