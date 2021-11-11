#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <franka/robot.h>
#include <franka/model.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

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

Eigen::Matrix<double, 9, 1> extend(const Eigen::Matrix<double, 7, 1> &v)
{
    Eigen::Matrix<double, 9, 1> r;
    r.block<7,1>(0,0) = v;
    r.block<2,1>(7,0) = Eigen::Matrix<double, 2, 1>::Zero();
    return r;
}

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

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./franka_emulator_calibrator <IP>" << std::endl;
        return 1;
    }

    //Init robot
    franka::Robot real_robot(argv[1]);
    franka::Model real_model = real_robot.loadModel();

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
            cas.gravity = vector(real_model.gravity(array(q), 0.0, std::array<double, 3>({0,0,0}), array(cas.g)));
            gravity_cases.push_back(cas);
        }
        for (int c = 0; c < 7; c++)
        {
            CoriolisCase cas;
            cas.q = q;
            cas.dq(c) = 1.0;
            cas.coriolis = vector(real_model.coriolis(array(q), array(cas.dq), std::array<double, 9>({0,0,0,0,0,0,0,0,0}), 0.0, std::array<double, 3>({0,0,0})));
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
            emulator_model.gravity.linear_impl() = gravity_cases[cas].g;
            Eigen::Matrix<double, 7, 1> emulator_gravity = pinocchio::computeGeneralizedGravity(emulator_model, emulator_data, extend(gravity_cases[cas].q)).block<7,1>(0,0);
            error.block<7,1>(gravity_error0 + 7*cas,0) = emulator_gravity - gravity_cases[cas].gravity;
        }
        std::cout << "Computing error of coriolis()" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
        {
            Eigen::Matrix<double, 7, 7> emulator_coriolis_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
            Eigen::Matrix<double, 7, 1> emulator_coriolis = emulator_coriolis_matrix * coriolis_cases[cas].dq;
            error.block<7,1>(coriolis_error0 + 7*cas,0) = emulator_coriolis - coriolis_cases[cas].coriolis;
        }
        std::cout << "Error norm: " << error.norm() << std::endl;
        if (error.norm() < 1.0) break;

        //Computing derivative
        std::cout << "Computing derivative of gravity() in mass" << std::endl;
        for (unsigned int cas = 0; cas < gravity_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
            {
                const double mass_epsillon = 0.1;
                double previous_mass = emulator_model.inertias[link+1].mass();
                emulator_model.gravity.linear_impl() = gravity_cases[cas].g;
                emulator_model.inertias[link+1].mass() = previous_mass + mass_epsillon;
                Eigen::Matrix<double, 7, 1> upper = pinocchio::computeGeneralizedGravity(emulator_model, emulator_data, extend(gravity_cases[cas].q)).block<7,1>(0,0);
                emulator_model.inertias[link+1].mass() = previous_mass - mass_epsillon;
                Eigen::Matrix<double, 7, 1> lower = pinocchio::computeGeneralizedGravity(emulator_model, emulator_data, extend(gravity_cases[cas].q)).block<7,1>(0,0);
                emulator_model.inertias[link+1].mass() = previous_mass;
                derivative.block<7,1>(gravity_error0 + 7*cas, nparameters*link + mass0) = (upper - lower) / (2 * mass_epsillon);
            }

        std::cout << "Computing derivative of gravity() in position" << std::endl;
        for (unsigned int cas = 0; cas < gravity_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
                for (unsigned int position = 0; position < 3; position++)
                {
                    const double position_epsillon = 0.1;
                    double previous_position = emulator_model.inertias[link+1].lever()[position];
                    emulator_model.gravity.linear_impl() = gravity_cases[cas].g;
                    emulator_model.inertias[link+1].lever()[position] = previous_position + position_epsillon;
                    Eigen::Matrix<double, 7, 1> upper = pinocchio::computeGeneralizedGravity(emulator_model, emulator_data, extend(gravity_cases[cas].q)).block<7,1>(0,0);
                    emulator_model.inertias[link+1].lever()[position] = previous_position - position_epsillon;
                    Eigen::Matrix<double, 7, 1> lower = pinocchio::computeGeneralizedGravity(emulator_model, emulator_data, extend(gravity_cases[cas].q)).block<7,1>(0,0);
                    emulator_model.inertias[link+1].lever()[position] = previous_position;
                    derivative.block<7,1>(gravity_error0 + 7*cas, nparameters*link + position0 + position) = (upper - lower) / (2 * position_epsillon);
                }
        
        std::cout << "Computing derivative of coriolis() in mass" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
            {
                const double mass_epsillon = 0.1;
                double previous_mass = emulator_model.inertias[link+1].mass();
                emulator_model.inertias[link+1].mass() = previous_mass + mass_epsillon;
                Eigen::Matrix<double, 7, 7> upper_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
                Eigen::Matrix<double, 7, 1> upper = upper_matrix * coriolis_cases[cas].dq;
                emulator_model.inertias[link+1].mass() = previous_mass - mass_epsillon;
                Eigen::Matrix<double, 7, 7> lower_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
                Eigen::Matrix<double, 7, 1> lower = upper_matrix * coriolis_cases[cas].dq;
                emulator_model.inertias[link+1].mass() = previous_mass;
                derivative.block<7,1>(coriolis_error0 + 7*cas, nparameters*link + mass0) = (upper - lower) / (2 * mass_epsillon);
            }

        std::cout << "Computing derivative of coriolis() in inertia" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
                for (unsigned int inertia = 0; inertia < 6; inertia++)
                {
                    const double inertia_epsillon = 0.1;
                    double previous_inertia = emulator_model.inertias[link+1].inertia().data()[inertia];
                    emulator_model.inertias[link+1].inertia().data()[inertia] = previous_inertia + inertia_epsillon;
                    Eigen::Matrix<double, 7, 7> upper_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
                    Eigen::Matrix<double, 7, 1> upper = upper_matrix * coriolis_cases[cas].dq;
                    emulator_model.inertias[link+1].inertia().data()[inertia] = previous_inertia - inertia_epsillon;
                    Eigen::Matrix<double, 7, 7> lower_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
                    Eigen::Matrix<double, 7, 1> lower = upper_matrix * coriolis_cases[cas].dq;
                    emulator_model.inertias[link+1].inertia().data()[inertia] = previous_inertia;
                    derivative.block<7,1>(coriolis_error0 + 7*cas, nparameters*link + inertia0 + inertia) = (upper - lower) / (2 * inertia_epsillon);
                }
        
        std::cout << "Computing derivative of coriolis() in position" << std::endl;
        for (unsigned int cas = 0; cas < coriolis_cases.size(); cas++)
            for (unsigned int link = 0; link < 7; link++)
                for (unsigned int position = 0; position < 3; position++)
                {
                    const double position_epsillon = 0.1;
                    double previous_position = emulator_model.inertias[link+1].lever()[position];
                    emulator_model.inertias[link+1].lever()[position] = previous_position + position_epsillon;
                    Eigen::Matrix<double, 7, 7> upper_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
                    Eigen::Matrix<double, 7, 1> upper = upper_matrix * coriolis_cases[cas].dq;
                    emulator_model.inertias[link+1].lever()[position] = previous_position - position_epsillon;
                    Eigen::Matrix<double, 7, 7> lower_matrix = pinocchio::computeCoriolisMatrix(emulator_model, emulator_data, extend(coriolis_cases[cas].q), extend(coriolis_cases[cas].dq)).block<7,7>(0,0);
                    Eigen::Matrix<double, 7, 1> lower = upper_matrix * coriolis_cases[cas].dq;
                    emulator_model.inertias[link+1].lever()[position] = previous_position;
                    derivative.block<7,1>(coriolis_error0 + 7*cas, nparameters*link + position0 + position) = (upper - lower) / (2 * position_epsillon);
                }
        
        //Updating parameters
        std::cout << "Computing parameters for next step" << std::endl;
        parameters -= derivative.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(error);
        std::cout << "Applying parameters" << std::endl;
        for (unsigned int link = 0; link < 7; link++)
        {
            emulator_model.inertias[link+1].mass() = parameters(nparameters*link + mass0);
            for (unsigned int inertia = 0; inertia < 6; inertia++) emulator_model.inertias[link+1].inertia().data()[inertia] = parameters(nparameters*link + inertia0 + inertia);
            for (unsigned int position = 0; position < 3; position++) emulator_model.inertias[link+1].lever()[position] = parameters(nparameters*link + position0 + position);
        }
    }

    //Printing parameters
    std::cout << "Calibration finished" << std::endl;
    for (unsigned int link = 0; link < 7; link++)
    {
        std::cout << "Link    : " << link << std::endl;
        std::cout << "Mass    : " << parameters(nparameters*link + mass0) << std::endl;
        std::cout << "Inertia :";
        for (unsigned int inertia = 0; inertia < 6; inertia++) std::cout << " " << parameters(nparameters*link + inertia0 + inertia);
        std::cout << std::endl;
        std::cout << "Position:";
        for (unsigned int position = 0; position < 3; position++) std::cout << " " << parameters(nparameters*link + position0 + position);
        std::cout << std::endl << std::endl;
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