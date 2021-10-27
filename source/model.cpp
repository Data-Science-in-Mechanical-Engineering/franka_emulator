#include "../include/franka/model.h"
#include <Eigen/Geometry>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

franka::Frame franka::operator++(Frame& frame, int /* dummy */) noexcept
{
    return static_cast<Frame>(static_cast<int>(frame) + 1);
}

franka::Model::Model(franka::Network&)
{
    pinocchio::urdf::buildModel("../model/franka.urdf", _model);
    _data = pinocchio::Data(_model);
    for (size_t i = 0; i < 7; i++)
    {
        std::string name = "panda_joint" + std::to_string(i + 1);
        if (!_model.existJointName(name)) throw std::runtime_error("Link not found");
        _joint_frame_id[i] = _model.getFrameId(name);
    }

    for (size_t i = 0; i < 7; i++)
    {
        std::string name = "panda_link" + std::to_string(i + 1);
        if (!_model.existFrame(name)) throw std::runtime_error("Link not found");
        _link_frame_id[i] = _model.getFrameId(name);
    }
}

franka::Model::Model(Model &&other) noexcept
{
    _model = other._model;
    _data = other._data;
    for (size_t i = 0; i < 7; i++) _joint_frame_id[i] = other._joint_frame_id[i];
    for (size_t i = 0; i < 7; i++) _link_frame_id[i] = other._link_frame_id[i];
}

franka::Model& franka::Model::operator=(Model&&) noexcept
{
    return *this;
}

franka::Model::~Model() noexcept
{}

std::array<double, 16> franka::Model::pose(Frame frame, const franka::RobotState& robot_state) const
{
    return pose(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 16> franka::Model::pose(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    Eigen::Vector<double, 9> full_q;
    full_q.block<7, 1>(0, 0) = Eigen::Vector<double, 7>::Map(&q[0]);
    full_q.block<2, 1>(7, 0) = Eigen::Vector<double, 2>::Zero();
    pinocchio::forwardKinematics(_model, _data, full_q);
    Eigen::Affine3d transform;
    if (static_cast<size_t>(frame) >= static_cast<size_t>(Frame::kJoint1) && static_cast<size_t>(frame) <= static_cast<size_t>(Frame::kJoint7))
    {
        pinocchio::SE3 se3 = _data.oMf[_link_frame_id[static_cast<size_t>(frame) - static_cast<size_t>(Frame::kJoint1)]];
        transform.linear() = se3.rotation_impl();
        transform.translation() = se3.translation_impl();        
    }
    else if (frame == Frame::kEndEffector)
    {
        pinocchio::SE3 se3 = _data.oMf[_link_frame_id[6]];
        transform.linear() = se3.rotation_impl();
        transform.translation() = se3.translation_impl() + 0.210399 * transform.linear().col(2);
    }
    else throw std::runtime_error("franka::Model::pose: Unknown frame");
    std::array<double, 16> result;
    Eigen::Matrix4d::Map(&result[0]) = transform.matrix();
    return result;
}

std::array<double, 42> franka::Model::bodyJacobian(Frame frame, const franka::RobotState& robot_state) const
{
    return bodyJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 42> franka::Model::bodyJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    if (static_cast<size_t>(frame) < static_cast<size_t>(Frame::kJoint1) || static_cast<size_t>(frame) > static_cast<size_t>(Frame::kJoint7)) throw std::runtime_error("franka::Model::pose: Unknown frame");
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = Eigen::Matrix<double, 7, 1>::Map(&q[0]);
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 6, 9> full_result;
    pinocchio::computeFrameJacobian(_model, _data, full_q, _joint_frame_id[static_cast<size_t>(frame) - static_cast<size_t>(Frame::kJoint1)], pinocchio::ReferenceFrame::LOCAL, full_result);
    std::array<double, 42> result;
    Eigen::Matrix<double, 6, 7>::Map(&result[0]) = full_result.block<6, 7>(0, 0);
    return result;
}

std::array<double, 42> franka::Model::zeroJacobian(Frame frame, const franka::RobotState& robot_state) const
{
    return zeroJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 42> franka::Model::zeroJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    if (static_cast<size_t>(frame) < static_cast<size_t>(Frame::kJoint1) || static_cast<size_t>(frame) > static_cast<size_t>(Frame::kJoint7)) throw std::runtime_error("franka::Model::pose: Unknown frame");
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = Eigen::Matrix<double, 7, 1>::Map(&q[0]);
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 6, 9> full_result;
    pinocchio::computeFrameJacobian(_model, _data, full_q, _joint_frame_id[static_cast<size_t>(frame) - static_cast<size_t>(Frame::kJoint1)], pinocchio::ReferenceFrame::WORLD, full_result);
    std::array<double, 42> result;
    Eigen::Matrix<double, 6, 7>::Map(&result[0]) = full_result.block<6, 7>(0, 0);
    return result;
}

std::array<double, 49> franka::Model::mass(const franka::RobotState& robot_state) const noexcept
{
    return mass(robot_state.q, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array<double, 49> franka::Model::mass(
    const std::array<double, 7>& q,
    const std::array<double, 9>& I_total,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) const noexcept
{
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = Eigen::Matrix<double, 7, 1>::Map(&q[0]);
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 9, 9> full_result = pinocchio::crba(_model, _data, full_q);
    full_result.triangularView<Eigen::StrictlyLower>() = full_result.transpose().triangularView<Eigen::StrictlyLower>();
    std::array<double, 49> result;
    Eigen::Matrix<double, 7, 7>::Map(&result[0]) = full_result.block<7, 7>(0, 0);
    return result;
}

std::array<double, 7> franka::Model::coriolis(const franka::RobotState& robot_state) const noexcept
{
    return coriolis(robot_state.q, robot_state.dq, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array<double, 7> franka::Model::coriolis(
    const std::array<double, 7>& q,
    const std::array<double, 7>& dq,
    const std::array<double, 9>& I_total,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) const noexcept
{
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = Eigen::Matrix<double, 7, 1>::Map(&q[0]);
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 9, 1> full_dq;
    full_dq.block<7, 1>(0, 0) = Eigen::Matrix<double, 7, 1>::Map(&dq[0]);
    full_dq.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 9, 9> full_result = pinocchio::computeCoriolisMatrix(_model, _data, full_q, full_dq);
    std::array<double, 7> result;
    Eigen::Matrix<double, 7, 1>::Map(&result[0]) = (full_result * full_dq).block<7, 1>(0, 0);
    return result;
}

std::array<double, 7> franka::Model::gravity(
    const std::array<double, 7>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,
    const std::array<double, 3>& gravity_earth) const noexcept
{
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = Eigen::Matrix<double, 7, 1>::Map(&q[0]);
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 9, 1> full_result = pinocchio::computeGeneralizedGravity(_model, _data, full_q);
    std::array<double, 7> result;
    Eigen::Matrix<double, 7, 1>::Map(&result[0]) = full_result.block<7, 1>(0, 0);
    return result;
}

std::array<double, 7> franka::Model::gravity(
    const franka::RobotState& robot_state,
    const std::array<double, 3>& gravity_earth) const noexcept
{
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, gravity_earth);
}