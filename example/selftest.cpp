#include "../include/franka/model.h"
#include <franka/model.h>
#include <gtest/gtest.h>
#include <iostream>

static const double tolerance = 0.001;
static std::string real_ip;
franka::Model *real_model;
FRANKA_EMULATOR_CXX_NAME::Model *emulator_model;

namespace FRANKA_EMULATOR_CXX_NAME
{
    class Network{};
}

TEST(Model, Gravity)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    double m = 0.0;
    std::array<double, 3> F = { 0.0, 0.0, 0.0 };
    std::array<double, 7> real_gravity;
    EXPECT_NO_THROW(real_gravity = real_model->gravity(q, m, F));
    std::array<double, 7> emulator_gravity;
    EXPECT_NO_THROW(emulator_gravity = emulator_model->gravity(q, m, F));
    for (size_t i = 0; i < 7; i++) EXPECT_NEAR(real_gravity[i], emulator_gravity[i], tolerance);
}

TEST(Model, Coriolis)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    std::array<double, 7> dq = { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7 };
    std::array<double, 9> I = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    double m = 0.0;
    std::array<double, 3> F = { 0.0, 0.0, 0.0 };
    std::array<double, 7> real_coriolis;
    EXPECT_NO_THROW(real_coriolis = real_model->coriolis(q, dq, I, m, F));
    std::array<double, 7> emulator_coriolis;
    EXPECT_NO_THROW(emulator_coriolis = emulator_model->coriolis(q, dq, I, m, F));
    for (size_t i = 0; i < 7; i++) EXPECT_NEAR(real_coriolis[i], emulator_coriolis[i], tolerance);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    
    std::cout << "Enter real robot IP: ";
    std::getline(std::cin, real_ip);
    franka::Robot real_robot(real_ip);
    real_model = new franka::Model(real_robot.loadModel());
    FRANKA_EMULATOR_CXX_NAME::Network emulator_network;
    emulator_model = new FRANKA_EMULATOR_CXX_NAME::Model(emulator_network);

    return RUN_ALL_TESTS();
}