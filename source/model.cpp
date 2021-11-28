#include "../include/franka_emulator/model.h"
#include <Eigen/Geometry>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#ifndef _GNU_SOURCE
    #define _GNU_SOURCE
#endif
#include <link.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

FRANKA_EMULATOR::Frame FRANKA_EMULATOR::operator++(Frame& frame, int /* dummy */) noexcept
{
    Frame result = frame;
    frame = static_cast<Frame>(static_cast<unsigned int>(result) + 1);
    return result;
}

FRANKA_EMULATOR::Model::Model(FRANKA_EMULATOR::Network&)
{
    //Searching for urdf file
    struct LibrarySearchCallbackData
    {
        bool found              = false;
        std::string directory   = "";
    } library_search_callback_data;
    auto library_search_callback = [](struct dl_phdr_info *info, size_t size, void *data) -> int
    {
        const char *library_name = "libfranka_emulator_model.so";
        if (strstr(info->dlpi_name, library_name) != nullptr)
        {
            LibrarySearchCallbackData *library_search_callback_data = (LibrarySearchCallbackData*)data;
            library_search_callback_data->found = true;
            library_search_callback_data->directory.assign(info->dlpi_name);
            library_search_callback_data->directory.erase(library_search_callback_data->directory.find(library_name), strlen(library_name));
        }
        return 0;
    };
    dl_iterate_phdr(library_search_callback, &library_search_callback_data);
    if (!library_search_callback_data.found) throw std::runtime_error("franka_emulator::Model::Model: Could not find library path");
    struct stat model_stat;
    if (stat((library_search_callback_data.directory + "model/franka_emulator.urdf").c_str(), &model_stat) == 0)
        pinocchio::urdf::buildModel(library_search_callback_data.directory + "model/franka_emulator.urdf", _model);
    if (stat((library_search_callback_data.directory + "../model/franka_emulator.urdf").c_str(), &model_stat) == 0)
        pinocchio::urdf::buildModel(library_search_callback_data.directory + "../model/franka_emulator.urdf", _model);
    else if (stat((library_search_callback_data.directory + "../share/franka_emulator/model/franka_emulator.urdf").c_str(), &model_stat) == 0)
        pinocchio::urdf::buildModel(library_search_callback_data.directory + "../share/franka_emulator/model/franka_emulator.urdf", _model);
    else throw std::runtime_error("franka_emulator::Model::Model: Could not find model file");

    //Other initialization
    _data = pinocchio::Data(_model);
    for (size_t joint = 0; joint < 8; joint++)
    {
        std::string name = "panda_joint" + std::to_string(joint + 1);
        if (!_model.existFrame(name)) throw std::runtime_error("Joint frame not found");
        _joint_frame_id[joint] = _model.getFrameId(name);
    }
    for (size_t link = 0; link < 8; link++)
    {
        std::string name = "panda_link" + std::to_string(link + 1);
        if (!_model.existFrame(name)) throw std::runtime_error("Link frame not found");
        _link_frame_id[link] = _model.getFrameId(name);
    }
}

FRANKA_EMULATOR::Model::Model(Model &&other) noexcept
{
    _model = other._model;
    _data = other._data;
    for (size_t i = 0; i < 8; i++) _joint_frame_id[i] = other._joint_frame_id[i];
    for (size_t i = 0; i < 8; i++) _link_frame_id[i] = other._link_frame_id[i];
}

FRANKA_EMULATOR::Model& FRANKA_EMULATOR::Model::operator=(Model&&) noexcept
{
    return *this;
}

FRANKA_EMULATOR::Model::~Model() noexcept
{}

std::array<double, 16> FRANKA_EMULATOR::Model::pose(Frame frame, const FRANKA_EMULATOR::RobotState& robot_state) const
{
    return pose(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 16> FRANKA_EMULATOR::Model::pose(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    
    pinocchio::forwardKinematics(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]));
    pinocchio::SE3 placement;
    if (static_cast<size_t>(frame) >= static_cast<size_t>(Frame::kJoint1) && static_cast<size_t>(frame) <= static_cast<size_t>(Frame::kJoint7))
    {
        placement = _data.oMi[static_cast<size_t>(frame) + 1];
    }
    else if (frame == Frame::kFlange || frame == Frame::kEndEffector || frame == Frame::kStiffness)
    {
        placement = pinocchio::updateFramePlacement(_model, _data, _joint_frame_id[7]);
        if (frame == Frame::kEndEffector || frame == Frame::kStiffness) placement = placement.act_impl(pinocchio::SE3(Eigen::Matrix4d::Map(&F_T_EE[0])));
        if (frame == Frame::kStiffness) placement = placement.act_impl(pinocchio::SE3(Eigen::Matrix4d::Map(&EE_T_K[0])));
    }
    else throw std::runtime_error("franka_emulator::Model::pose: Invalid frame");

    Eigen::Affine3d transform;
    transform.linear() = placement.rotation_impl();
    transform.translation() = placement.translation_impl();    
    std::array<double, 16> result;
    Eigen::Matrix4d::Map(&result[0]) = transform.matrix();
    return result;
}

std::array<double, 42> FRANKA_EMULATOR::Model::bodyJacobian(Frame frame, const FRANKA_EMULATOR::RobotState& robot_state) const
{
    return bodyJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 42> FRANKA_EMULATOR::Model::bodyJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    Eigen::Matrix<double, 6, 7> result_matrix = Eigen::Matrix<double, 6, 7>::Zero();
    if (static_cast<size_t>(frame) >= static_cast<size_t>(Frame::kJoint1) && static_cast<size_t>(frame) <= static_cast<size_t>(Frame::kJoint7))
    {
        pinocchio::computeJointJacobian(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]), static_cast<size_t>(frame) + 1, result_matrix);
    }
    else if (frame == Frame::kFlange)
    {
        pinocchio::computeFrameJacobian(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]), _joint_frame_id[7], result_matrix);
    }
    else if (frame == Frame::kEndEffector || frame == Frame::kStiffness)
    {
        pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _joint_frame_id[7]).act_impl(pinocchio::SE3(Eigen::Matrix4d::Map(&F_T_EE[0])));
        if (frame == Frame::kStiffness) placement = placement.act_impl(pinocchio::SE3(Eigen::Matrix4d::Map(&EE_T_K[0])));
        for (size_t i = 0; i < 7; i++)
        {
            result_matrix.block<3, 1>(0, i) = placement.rotation_impl().transpose() * (_data.oMi[i+1].rotation_impl().col(2).cross(placement.translation_impl() - _data.oMi[1+i].translation_impl()));
            result_matrix.block<3, 1>(3, i) = placement.rotation_impl().transpose() * _data.oMi[i+1].rotation_impl().col(2);
        }
    }
    else throw std::runtime_error("franka_emulator::Model::bodyJacobian: Invalid frame");
    
    std::array<double, 42> result;
    Eigen::Matrix<double, 6, 7>::Map(&result[0]) = result_matrix;
    return result;
}

std::array<double, 42> FRANKA_EMULATOR::Model::zeroJacobian(Frame frame, const FRANKA_EMULATOR::RobotState& robot_state) const
{
    return zeroJacobian(frame, robot_state.q, robot_state.F_T_EE, robot_state.EE_T_K);
}

std::array<double, 42> FRANKA_EMULATOR::Model::zeroJacobian(
    Frame frame,
    const std::array<double, 7>& q,
    const std::array<double, 16>& F_T_EE,
    const std::array<double, 16>& EE_T_K) const
{
    Eigen::Matrix<double, 6, 7> result_matrix = Eigen::Matrix<double, 6, 7>::Zero();
    if (static_cast<size_t>(frame) >= static_cast<size_t>(Frame::kJoint1) && static_cast<size_t>(frame) <= static_cast<size_t>(Frame::kJoint7))
    {
        pinocchio::forwardKinematics(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]));
        pinocchio::computeJointJacobian(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]), static_cast<size_t>(frame) + 1, result_matrix);
        result_matrix.block<3, 7>(0,0) = _data.oMi[static_cast<size_t>(frame) + 1].rotation_impl() * result_matrix.block<3, 7>(0,0);
        result_matrix.block<3, 7>(3,0) = _data.oMi[static_cast<size_t>(frame) + 1].rotation_impl() * result_matrix.block<3, 7>(3,0);
    }
    else if (frame == Frame::kFlange)
    {
        pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _joint_frame_id[7]);
        pinocchio::computeFrameJacobian(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]), _joint_frame_id[7], result_matrix);
        result_matrix.block<3, 7>(0,0) = placement.rotation_impl() * result_matrix.block<3, 7>(0,0);
        result_matrix.block<3, 7>(3,0) = placement.rotation_impl() * result_matrix.block<3, 7>(3,0);
    }
    else if (frame == Frame::kEndEffector || frame == Frame::kStiffness)
    {
        pinocchio::SE3 placement = pinocchio::updateFramePlacement(_model, _data, _joint_frame_id[7]).act_impl(pinocchio::SE3(Eigen::Matrix4d::Map(&F_T_EE[0])));
        if (frame == Frame::kStiffness) placement = placement.act_impl(pinocchio::SE3(Eigen::Matrix4d::Map(&EE_T_K[0])));
        for (size_t i = 0; i < 7; i++)
        {
            result_matrix.block<3, 1>(0, i) = _data.oMi[i+1].rotation_impl().col(2).cross(placement.translation_impl() - _data.oMi[i+1].translation_impl());
            result_matrix.block<3, 1>(3, i) = _data.oMi[i+1].rotation_impl().col(2);
        }
    }
    else throw std::runtime_error("franka_emulator::Model::zeroJacobian: Invalid frame");

    std::array<double, 42> result;
    Eigen::Matrix<double, 6, 7>::Map(&result[0]) = result_matrix;
    return result;
}

std::array<double, 49> FRANKA_EMULATOR::Model::mass(const FRANKA_EMULATOR::RobotState& robot_state) const noexcept
{
    return mass(robot_state.q, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array<double, 49> FRANKA_EMULATOR::Model::mass(
    const std::array<double, 7>& q,
    const std::array<double, 9>& I_total,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) const noexcept
{
    pinocchio::Inertia initial_end_inertia = _model.inertias[7];
    _model.inertias[7].mass() += m_total;
    _model.inertias[7].inertia().data()[0] += I_total[0];                    //xx
    _model.inertias[7].inertia().data()[1] += (I_total[1] + I_total[3]) / 2; //xy
    _model.inertias[7].inertia().data()[2] += I_total[4];                    //yy
    _model.inertias[7].inertia().data()[3] += (I_total[2] + I_total[6]) / 2; //xz
    _model.inertias[7].inertia().data()[4] += (I_total[5] + I_total[7]) / 2; //yz
    _model.inertias[7].inertia().data()[5] += I_total[8];                    //zz
    Eigen::Matrix<double, 7, 7> result_matrix = pinocchio::crba(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]));
    _model.inertias[7] = initial_end_inertia;
    result_matrix.triangularView<Eigen::StrictlyLower>() = result_matrix.transpose().triangularView<Eigen::StrictlyLower>();
    std::array<double, 49> result;
    Eigen::Matrix<double, 7, 7>::Map(&result[0]) = result_matrix;
    return result;
}

std::array<double, 7> FRANKA_EMULATOR::Model::coriolis(const FRANKA_EMULATOR::RobotState& robot_state) const noexcept
{
    return coriolis(robot_state.q, robot_state.dq, robot_state.I_total, robot_state.m_total, robot_state.F_x_Ctotal);
}

std::array<double, 7> FRANKA_EMULATOR::Model::coriolis(
    const std::array<double, 7>& q,
    const std::array<double, 7>& dq,
    const std::array<double, 9>& I_total,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal) const noexcept
{
    pinocchio::Inertia initial_end_inertia = _model.inertias[7];
    _model.inertias[7].mass() += m_total;
    _model.inertias[7].inertia().data()[0] += I_total[0];                    //xx
    _model.inertias[7].inertia().data()[1] += (I_total[1] + I_total[3]) / 2; //xy
    _model.inertias[7].inertia().data()[2] += I_total[4];                    //yy
    _model.inertias[7].inertia().data()[3] += (I_total[2] + I_total[6]) / 2; //xz
    _model.inertias[7].inertia().data()[4] += (I_total[5] + I_total[7]) / 2; //yz
    _model.inertias[7].inertia().data()[5] += I_total[8];                    //zz
    Eigen::Matrix<double, 7, 7> result_matrix = pinocchio::computeCoriolisMatrix(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]), Eigen::Matrix<double, 7, 1>::Map(&dq[0]));
    _model.inertias[7] = initial_end_inertia;
    std::array<double, 7> result;
    Eigen::Matrix<double, 7, 1>::Map(&result[0]) = result_matrix * Eigen::Matrix<double, 7, 1>::Map(&dq[0]);
    return result;
}

std::array<double, 7> FRANKA_EMULATOR::Model::gravity(
    const std::array<double, 7>& q,
    double m_total,
    const std::array<double, 3>& F_x_Ctotal,
    const std::array<double, 3>& gravity_earth) const noexcept
{
    _model.gravity.linear_impl() = Eigen::Matrix<double, 3, 1>::Map(&gravity_earth[0]);
    double initial_end_mass = _model.inertias[7].mass();
    _model.inertias[7].mass() += m_total;
    std::array<double, 7> result;
    Eigen::Matrix<double, 7, 1>::Map(&result[0]) = pinocchio::computeGeneralizedGravity(_model, _data, Eigen::Matrix<double, 7, 1>::Map(&q[0]));
    _model.inertias[7].mass() = initial_end_mass;
    return result;
}

std::array<double, 7> FRANKA_EMULATOR::Model::gravity(
    const FRANKA_EMULATOR::RobotState& robot_state,
    const std::array<double, 3>& gravity_earth) const noexcept
{
    return gravity(robot_state.q, robot_state.m_total, robot_state.F_x_Ctotal, gravity_earth);
}