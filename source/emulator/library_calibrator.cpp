#include <franka/robot.h>
#include <franka/model.h>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <Eigen/Dense>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <iostream>
#include <limits>
#include <algorithm>

struct GravityCase
{
    Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 3, 1> g = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 7, 1> gravity = Eigen::Matrix<double, 7, 1>::Zero();
};

struct CoriolisCase
{
    Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> coriolis = Eigen::Matrix<double, 7, 1>::Zero();
};

std::array<double, 3> array(const Eigen::Matrix<double, 3, 1> &v)
{
    std::array<double, 3> r;
    Eigen::Matrix<double, 3, 1>::Map(&r[0]) = v;
    return r;
}

std::array<double, 7> array(const Eigen::Matrix<double, 7, 1> &v)
{
    std::array<double, 7> r;
    Eigen::Matrix<double, 7, 1>::Map(&r[0]) = v;
    return r;
}

Eigen::Matrix<double, 7, 1> vector(const std::array<double, 7> &v)
{
    return Eigen::Matrix<double, 7, 1>::Map(&v[0]);
}

Eigen::Matrix<double, 7, 1> compute_emulator_gravity(pinocchio::Model &model, pinocchio::Data &data, const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 3, 1> &gravity)
{
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = q;
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    pinocchio::forwardKinematics(model, data, full_q);
    pinocchio::computeSubtreeMasses(model, data);
    pinocchio::centerOfMass(model, data, full_q);
    Eigen::Matrix<double, 7, 1> result;
    for (size_t i = 0; i < 7; i++)
    {
        Eigen::Matrix<double, 3, 1> joint_to_com = data.oMi[i+1].act_impl(data.com[i+1]) - data.oMi[i+1].translation_impl();
        Eigen::Matrix<double, 3, 1> force = data.mass[i+1] * gravity;
        Eigen::Matrix<double, 3, 1> joint_axis = data.oMi[i+1].rotation_impl().col(2);
        result(i) = (joint_to_com.cross(force)).dot(joint_axis);
    }
    return result;
}

Eigen::Matrix<double, 7, 1> compute_emulator_coriolis(pinocchio::Model &model, pinocchio::Data &data, const Eigen::Matrix<double, 7, 1> &q, const Eigen::Matrix<double, 7, 1> &dq)
{
    Eigen::Matrix<double, 9, 1> full_q;
    full_q.block<7, 1>(0, 0) = q;
    full_q.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 9, 1> full_dq;
    full_dq.block<7, 1>(0, 0) = dq;
    full_dq.block<2, 1>(7, 0) = Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 9, 9> full_result = pinocchio::computeCoriolisMatrix(model, data, full_q, full_dq);
    return (full_result * full_dq).block<7, 1>(0, 0);
}

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./franka_emulator_library_calibrator <IP>" << std::endl;
        return 1;
    }

    //Init robot
    franka::Robot real_robot(argv[1]);
    franka::Model real_model = real_robot.loadModel();
    franka::RobotState real_state = real_robot.readOnce();

    //Init model
    pinocchio::Model emulator_model;
    struct stat model_stat;
    if (stat("model/franka.urdf", &model_stat) == 0) pinocchio::urdf::buildModel("model/franka.urdf", emulator_model);
    else if (stat("../model/franka.urdf", &model_stat) == 0) pinocchio::urdf::buildModel("../model/franka.urdf", emulator_model);
    else throw std::runtime_error("Could not find model file");
    pinocchio::Data emulator_data(emulator_model);

    //Generating test cases
    std::vector<GravityCase> gravity_cases;
    std::vector<CoriolisCase> coriolis_cases;
    gravity_cases.reserve(3 * 3*3*3*3*3*3*3);
    coriolis_cases.reserve(7 * 3*3*3*3*3*3*3);
    for (int j0 = -1; j0 <= 1; j0++)
    for (int j1 = -1; j1 <= 1; j1++)
    for (int j2 = -1; j2 <= 1; j2++)
    for (int j3 = -1; j3 <= 1; j3++)
    for (int j4 = -1; j4 <= 1; j4++)
    for (int j5 = -1; j5 <= 1; j5++)
    for (int j6 = -1; j6 <= 1; j6++)
    {
        Eigen::Matrix<double, 7, 1> q;
        q(0) = j0 * M_PI / 2;
        q(1) = j1 * M_PI / 2;
        q(2) = j2 * M_PI / 2;
        q(3) = -M_PI/2 + j3 * M_PI/4;
        q(4) = j4 * M_PI / 2;
        q(5) = M_PI/2 + j5 * M_PI/2;
        q(6) = j6 * M_PI / 2;
        for (int g = 0; g < 3; g++)
        {
            GravityCase cas;
            cas.q = q;
            cas.g(g) = -10.0;
            cas.gravity = vector(real_model.gravity(array(q), real_state.m_total, std::array<double, 3>({0,0,0}), array(cas.g)));
            gravity_cases.push_back(cas);
        }
        for (int c = 0; c < 7; c++)
        {
            CoriolisCase cas;
            cas.q = q;
            cas.dq(c) = 1.0;
            cas.coriolis = vector(real_model.coriolis(array(q), array(cas.dq), real_state.I_total, real_state.m_total, std::array<double, 3>({0,0,0})));
            coriolis_cases.push_back(cas);
        }
    }

    //Extracting parametes
    const unsigned int nparameters = 10;
    const unsigned int mass0 = 0;
    const unsigned int inertia0 = 1;
    const unsigned int position0 = 7;
    Eigen::VectorXd parameters(7*nparameters); //7 links * (mass + 6 inertia + 3 position)
    for (unsigned int link = 0; link < 7; link++)
    {
        parameters(nparameters*link + mass0) = emulator_model.inertias[link+1].mass();
        for (unsigned int inertia = 0; inertia < 6; inertia++) parameters(nparameters*link + inertia0 + inertia) = emulator_model.inertias[link+1].inertia().data()[inertia];
        for (unsigned int position = 0; position < 3; position++) parameters(nparameters*link + position0 + position) = emulator_model.inertias[link+1].lever()[position];
    }
    Eigen::VectorXd previous_parameters = parameters;
    double previous_error_norm = std::numeric_limits<double>::quiet_NaN();

    //Run Newton method
    const unsigned int gravity_error0 = 0;
    const unsigned int coriolis_error0 = 7*gravity_cases.size();
    Eigen::VectorXd error(7*gravity_cases.size() + 7*coriolis_cases.size());
    Eigen::MatrixXd derivative(7*gravity_cases.size() + 7*coriolis_cases.size(), 7*nparameters);
    error.setZero();
    derivative.setZero();
    for (unsigned int step = 0;; step++)
    {
        //Computing error
        std::cout << "Calibration step " << step << std::endl;
        std::cout << "Computing error of gravity()" << std::endl;
        for (unsigned int cas = 0; cas < gravity_cases.size(); cas++)
        {
            Eigen::Matrix<double, 7, 1> emulator_gravity = compute_emulator_gravity(emulator_model, emulator_data, gravity_cases[cas].q, gravity_cases[cas].g);
            error.block<7,1>(gravity_error0 + 7*cas,0) = emulator_gravity - gravity_cases[cas].gravity;
        }
        std::cout << "Computing error of coriolis()" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
        {
            Eigen::Matrix<double, 7, 1> emulator_coriolis = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
            error.block<7,1>(coriolis_error0 + 7*cas,0) = emulator_coriolis - coriolis_cases[cas].coriolis;
        }
        std::cout << "Error norm: " << error.norm() << std::endl;
        if (error.norm() >= previous_error_norm) break;
        previous_parameters = parameters;
        previous_error_norm = error.norm();

        //Computing derivative
        std::cout << "Computing derivative of gravity() in mass" << std::endl;
        for (unsigned int cas = 0; cas < gravity_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
            {
                const double mass_epsillon = 0.01;
                double previous_mass = emulator_model.inertias[link+1].mass();
                emulator_model.inertias[link+1].mass() = previous_mass + mass_epsillon;
                Eigen::Matrix<double, 7, 1> upper = compute_emulator_gravity(emulator_model, emulator_data, gravity_cases[cas].q, gravity_cases[cas].g);
                emulator_model.inertias[link+1].mass() = std::max(previous_mass - mass_epsillon, 0.0);
                Eigen::Matrix<double, 7, 1> lower = compute_emulator_gravity(emulator_model, emulator_data, gravity_cases[cas].q, gravity_cases[cas].g);
                emulator_model.inertias[link+1].mass() = previous_mass;
                derivative.block<7,1>(gravity_error0 + 7*cas, nparameters*link + mass0) = (upper - lower) / (mass_epsillon + std::min(mass_epsillon, previous_mass));
            }

        std::cout << "Computing derivative of gravity() in position" << std::endl;
        for (unsigned int cas = 0; cas < gravity_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
                for (unsigned int position = 0; position < 3; position++)
                {
                    const double position_epsillon = 0.01;
                    double previous_position = emulator_model.inertias[link+1].lever()[position];
                    emulator_model.inertias[link+1].lever()[position] = previous_position + position_epsillon;
                    Eigen::Matrix<double, 7, 1> upper = compute_emulator_gravity(emulator_model, emulator_data, gravity_cases[cas].q, gravity_cases[cas].g);
                    emulator_model.inertias[link+1].lever()[position] = previous_position - position_epsillon;
                    Eigen::Matrix<double, 7, 1> lower = compute_emulator_gravity(emulator_model, emulator_data, gravity_cases[cas].q, gravity_cases[cas].g);
                    emulator_model.inertias[link+1].lever()[position] = previous_position;
                    derivative.block<7,1>(gravity_error0 + 7*cas, nparameters*link + position0 + position) = (upper - lower) / (2 * position_epsillon);
                }
        
        std::cout << "Computing derivative of coriolis() in mass" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
            {
                const double mass_epsillon = 0.01;
                double previous_mass = emulator_model.inertias[link+1].mass();
                emulator_model.inertias[link+1].mass() = previous_mass + mass_epsillon;
                Eigen::Matrix<double, 7, 1> upper = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
                emulator_model.inertias[link+1].mass() = std::max(previous_mass - mass_epsillon, 0.0);
                Eigen::Matrix<double, 7, 1> lower = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
                emulator_model.inertias[link+1].mass() = previous_mass;
                derivative.block<7,1>(coriolis_error0 + 7*cas, nparameters*link + mass0) = (upper - lower) / (mass_epsillon + std::min(mass_epsillon, previous_mass));
            }

        std::cout << "Computing derivative of coriolis() in inertia" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
                for (unsigned int inertia = 0; inertia < 6; inertia++)
                {
                    const double inertia_epsillon = 0.01;
                    bool central = inertia == 0 || inertia == 2 || inertia == 5;
                    double previous_inertia = emulator_model.inertias[link+1].inertia().data()[inertia];
                    emulator_model.inertias[link+1].inertia().data()[inertia] = previous_inertia + inertia_epsillon;
                    Eigen::Matrix<double, 7, 1> upper = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
                    emulator_model.inertias[link+1].inertia().data()[inertia] = (central ? std::max(previous_inertia - inertia_epsillon, 0.0) : (previous_inertia - inertia_epsillon));
                    Eigen::Matrix<double, 7, 1> lower = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
                    emulator_model.inertias[link+1].inertia().data()[inertia] = previous_inertia;
                    derivative.block<7,1>(coriolis_error0 + 7*cas, nparameters*link + inertia0 + inertia) = (upper - lower) / (inertia_epsillon + (central ? std::min(inertia_epsillon, previous_inertia) : inertia_epsillon));
                }
        
        std::cout << "Computing derivative of coriolis() in position" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
                for (unsigned int position = 0; position < 3; position++)
                {
                    const double position_epsillon = 0.01;
                    double previous_position = emulator_model.inertias[link+1].lever()[position];
                    emulator_model.inertias[link+1].lever()[position] = previous_position + position_epsillon;
                    Eigen::Matrix<double, 7, 1> upper = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
                    emulator_model.inertias[link+1].lever()[position] = previous_position - position_epsillon;
                    Eigen::Matrix<double, 7, 1> lower = compute_emulator_coriolis(emulator_model, emulator_data, coriolis_cases[cas].q, coriolis_cases[cas].dq);
                    emulator_model.inertias[link+1].lever()[position] = previous_position;
                    derivative.block<7,1>(coriolis_error0 + 7*cas, nparameters*link + position0 + position) = (upper - lower) / (2 * position_epsillon);
                }
        
        //Updating parameters
        std::cout << "Computing parameters for next step" << std::endl;
        parameters -= derivative.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(error);
        std::cout << "Applying parameters" << std::endl;
        for (unsigned int link = 0; link < 7; link++)
        {
            if (parameters(nparameters*link + mass0) < 0.0) parameters(nparameters*link + mass0) = 0.0;
            emulator_model.inertias[link+1].mass() = parameters(nparameters*link + mass0);
            for (unsigned int inertia = 0; inertia < 6; inertia++)
            {
                bool central = inertia == 0 || inertia == 2 || inertia == 5;
                if (central && parameters(nparameters*link + inertia0 + inertia) < 0.0) parameters(nparameters*link + inertia0 + inertia) = 0.0;
                emulator_model.inertias[link+1].inertia().data()[inertia] = parameters(nparameters*link + inertia0 + inertia);
            }
            for (unsigned int position = 0; position < 3; position++) emulator_model.inertias[link+1].lever()[position] = parameters(nparameters*link + position0 + position);
        }
    }

    //Printing parameters
    std::cout << "Calibration finished" << std::endl;
    for (unsigned int link = 0; link < 7; link++)
    {
        std::cout << "<link name=\"panda_link" << link+1 << "\">" << std::endl;

        std::cout << "<origin rpy=\"0 0 0\" xyz=\"";
        std::cout << previous_parameters(nparameters*link + position0) << " ";
        std::cout << previous_parameters(nparameters*link + position0 + 1) << " ";
        std::cout << previous_parameters(nparameters*link + position0 + 2) << "\"/>" << std::endl;

        std::cout << "<mass value=\"" << previous_parameters(nparameters*link + mass0) << "\"/>" << std::endl;
        
        std::cout << "<inertia ";
        std::cout << "ixx=\"" << previous_parameters(nparameters*link + inertia0) << "\" ";
        std::cout << "ixy=\"" << previous_parameters(nparameters*link + inertia0 + 1) << "\" ";
        std::cout << "ixz=\"" << previous_parameters(nparameters*link + inertia0 + 3) << "\" ";
        std::cout << "iyy=\"" << previous_parameters(nparameters*link + inertia0 + 2) << "\" ";
        std::cout << "iyz=\"" << previous_parameters(nparameters*link + inertia0 + 4) << "\" ";
        std::cout << "izz=\"" << previous_parameters(nparameters*link + inertia0 + 5) << "\"/>" << std::endl;
        
        std::cout << std::endl;
    }
    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return _main(argc, argv);
    }
    catch (std::exception &e)
    {
        std::cout << "Exception occured: " << e.what() << std::endl;
        return 1;
    }
}