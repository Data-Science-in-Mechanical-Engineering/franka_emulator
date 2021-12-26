#include "../include/franka_emulator/model.h"
#include "../include/franka_emulator/emulator/network.h"
#include "../include/franka_emulator/emulator/constants.h"
#if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    #include <franka/model.h>
#endif
#include <gtest/gtest.h>
#include <time.h>
#include <iostream>

static const double absolute_tolerance = 0.05;
static std::string real_ip;
#if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    franka::Robot *real_robot;
    franka::Model *real_model;
    franka::RobotState real_state;
#endif
FRANKA_EMULATOR::Model *emulator_model;
FRANKA_EMULATOR::RobotState emulator_state;

size_t timespec_difference(const timespec &a, const timespec &b)
{
    if (a.tv_sec == b.tv_sec && a.tv_nsec == b.tv_nsec) return 0;
    timespec first, second;
    if (a.tv_sec > b.tv_sec || (a.tv_sec == b.tv_sec && a.tv_nsec > b.tv_nsec)) { first = b; second = a; }
    else { first = a; second = b; }
    while (second.tv_nsec < first.tv_nsec) { second.tv_nsec += 1000*1000*1000; second.tv_sec--; }
    return 1000*1000*1000 * (second.tv_sec - first.tv_sec) + (second.tv_nsec - first.tv_nsec);
}

template<long unsigned int N> double array_difference(const std::array<double, N> &a, const std::array<double, N> &b)
{
    double result = 0;
    for (size_t i = 0; i < N; i++)
    {
        if (a[i] != a[i] || b[i] != b[i]) return std::numeric_limits<double>::quiet_NaN();
        else if (abs(a[i] - b[i]) > result) result = abs(a[i] - b[i]);
    }
    return result;
}

template<long unsigned int N> void array_print(const std::array<double, N> &a)
{
    for (size_t i = 0; i < (N-1); i++) std::cout << a[i] << ", ";
    std::cout << a[N - 1];
}

TEST(Model, PoseTime)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    //Real pose
    std::array<double, 16> real_pose;
    timespec real_start, real_finish;
    if (real_ip == "")
    {
        real_start.tv_sec = 0;
        real_start.tv_nsec = 0;
        real_finish.tv_sec = 0;
        real_finish.tv_nsec = 30479; //Intel(R) Xeon(R) W-2123 CPU @ 3.60GHz
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        clock_gettime(CLOCK_MONOTONIC, &real_start);
        EXPECT_NO_THROW(real_pose = real_model->pose(franka::Frame::kJoint1, q, real_state.F_T_EE, real_state.EE_T_K));
        clock_gettime(CLOCK_MONOTONIC, &real_finish);
    }
    #endif
    std::cout << "Real pose computed in " << 0.001 * timespec_difference(real_start, real_finish) << "us" << std::endl;
    
    //Emulator pose
    std::array<double, 16> emulator_pose;
    timespec emulator_start, emulator_finish;
    clock_gettime(CLOCK_MONOTONIC, &emulator_start);
    EXPECT_NO_THROW(emulator_pose = emulator_model->pose(FRANKA_EMULATOR::Frame::kJoint1, q, emulator_state.F_T_EE, emulator_state.EE_T_K));
    clock_gettime(CLOCK_MONOTONIC, &emulator_finish);
    std::cout << "Emulator pose computed in " << 0.001 * timespec_difference(emulator_start, emulator_finish) << "us" << std::endl;

    EXPECT_LE(timespec_difference(emulator_start, emulator_finish), 1000*1000);
}

TEST(Model, PoseAccuracy)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    //Real pose
    std::array<double, 16> real_pose[10];
    if (real_ip == "")
    {
        real_pose[0] = std::array<double, 16>({ 1, 0, 0, 0, -0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0.333, 1 });
        real_pose[1] = std::array<double, 16>({ 0.707107, 0, 0.707107, 0, 0.707107, 0, -0.707107, 0, -0, 1, 0, 0, 0, 0, 0.333, 1});
        real_pose[2] = std::array<double, 16>({ 0.707107, 0, 0.707107, 0, -0, 1, -0, 0, -0.707107, -0, 0.707107, 0, -0.223446, -0, 0.556446, 1 });
        real_pose[3] = std::array<double, 16>({ 0, 0, -1, 0, 1, 0, 0, 0, 0, -1, 0, 0, -0.165109, 0, 0.614782, 1 });
        real_pose[4] = std::array<double, 16>({ -0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0.218891, 0, 0.697282, 1 });
        real_pose[5] = std::array<double, 16>({ 1, 0, -6.12323e-17, 0, 6.12323e-17, 0, 1, 0, 0, -1, 0, 0, 0.218891, 0, 0.697282, 1 });
        real_pose[6] = std::array<double, 16>({ 0.707107, -0.707107, -4.32978e-17, 0, -0.707107, -0.707107, 4.32978e-17, 0, -6.12323e-17, 0, -1, 0, 0.306891, 0, 0.697282, 1 });
        real_pose[7] = std::array<double, 16>({ 0.707107, -0.707107, -4.32978e-17, 0, -0.707107, -0.707107, 4.32978e-17, 0, -6.12323e-17, 0, -1, 0, 0.306891, 0, 0.590282, 1 });
        real_pose[8] = std::array<double, 16>({ 0.99999, 5.55112e-17, -6.12318e-17, 0, 5.55112e-17, -0.99999, -6.16298e-33, 0, -6.12323e-17, 0, -1, 0, 0.306891, 0, 0.486882, 1 });
        real_pose[9] = std::array<double, 16>({ 0.99999, 5.55112e-17, -6.12318e-17, 0, 5.55112e-17, -0.99999, -6.16298e-33, 0, -6.12323e-17, 0, -1, 0, 0.306891, 0, 0.486882, 1 });
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        franka::Frame frame = franka::Frame::kJoint1;
        for (size_t i = 0; i < 10; i++, frame++) EXPECT_NO_THROW(real_pose[i] = real_model->pose(frame, q, real_state.F_T_EE, real_state.EE_T_K));
    }
    #endif
    std::cout << "Real poses:" << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        array_print(real_pose[i]);
        std::cout << std::endl;
    }
    
    //Emulator pose
    std::array<double, 16> emulator_pose[10];
    FRANKA_EMULATOR::Frame frame = FRANKA_EMULATOR::Frame::kJoint1;
    for (size_t i = 0; i < 10; i++, frame++) EXPECT_NO_THROW(emulator_pose[i] = emulator_model->pose(frame, q, emulator_state.F_T_EE, emulator_state.EE_T_K));
    std::cout << "Emulator poses:" << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        array_print(emulator_pose[i]);
        std::cout << std::endl;
    }

    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint1)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint1)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint2)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint2)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint3)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint3)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint4)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint4)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint5)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint5)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint6)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint6)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint7)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint7)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kFlange)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kFlange)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kEndEffector)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kEndEffector)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kStiffness)], emulator_pose[static_cast<int>(FRANKA_EMULATOR::Frame::kStiffness)]), absolute_tolerance);
}

TEST(Model, JacobianTime)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    //Real jacobian
    std::array<double, 42> real_jacobian;
    timespec real_start, real_finish;
    if (real_ip == "")
    {
        real_start.tv_sec = 0;
        real_start.tv_nsec = 0;
        real_finish.tv_sec = 0;
        real_finish.tv_nsec = 62217; //Intel(R) Xeon(R) W-2123 CPU @ 3.60GHz
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        clock_gettime(CLOCK_MONOTONIC, &real_start);
        EXPECT_NO_THROW(real_jacobian = real_model->zeroJacobian(franka::Frame::kJoint1, q, real_state.F_T_EE, real_state.EE_T_K));
        clock_gettime(CLOCK_MONOTONIC, &real_finish);
    }
    #endif
    std::cout << "Real jacobian computed in " << 0.001 * timespec_difference(real_start, real_finish) << "us" << std::endl;
    
    //Emulator jacobian
    std::array<double, 42> emulator_jacobian;
    timespec emulator_start, emulator_finish;
    clock_gettime(CLOCK_MONOTONIC, &emulator_start);
    EXPECT_NO_THROW(emulator_jacobian = emulator_model->zeroJacobian(FRANKA_EMULATOR::Frame::kJoint1, q, emulator_state.F_T_EE, emulator_state.EE_T_K));
    clock_gettime(CLOCK_MONOTONIC, &emulator_finish);
    std::cout << "Emulator jacobian computed in " << 0.001 * timespec_difference(emulator_start, emulator_finish) << "us" << std::endl;

    EXPECT_LE(timespec_difference(emulator_start, emulator_finish), 1000*1000);
}

TEST(Model, ZeroJacobianAccuracy)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    //Real jacobian
    std::array<double, 42> real_jacobian[10];
    if (real_ip == "")
    {
        real_jacobian[0] = std::array<double, 42>({ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[1] = std::array<double, 42>({ 0, 0, 0, 0, 0, 1, 0, 0, 0, -0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[2] = std::array<double, 42>({ 0, -0.223446, 0, 0, 0, 1, 0.223446, 0, 0.223446, -0, 1, 0, 0, 0, 0, -0.707107, -0, 0.707107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[3] = std::array<double, 42>({ 0, -0.165109, 0, 0, 0, 1, 0.281782, 0, 0.165109, -0, 1, 0, -0, 0.0825, -0, -0.707107, -0, 0.707107, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[4] = std::array<double, 42>({ 0, 0.218891, 0, 0, 0, 1, 0.364282, 0, -0.218891, -0, 1, 0, -0, 0.412365, -0, -0.707107, -0, 0.707107, -0.0825, 0, 0.384, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[5] = std::array<double, 42>({ 0, 0.218891, 0, 0, 0, 1, 0.364282, 0, -0.218891, -0, 1, 0, -0, 0.412365, -0, -0.707107, -0, 0.707107, -0.0825, 0, 0.384, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[6] = std::array<double, 42>({ 0, 0.306891, 0, 0, 0, 1, 0.364282, 0, -0.306891, -0, 1, 0, -0, 0.474591, -0, -0.707107, -0, 0.707107, -0.0825, 0, 0.472, 0, -1, 0, -0, 5.38845e-18, 0, 1, 0, 0, 5.38845e-18, 0, 0.088, 0, -1, 0, 0, 0, 0, -6.12323e-17, 0, -1 });
        real_jacobian[7] = std::array<double, 42>({ 0, 0.306891, 0, 0, 0, 1, 0.257282, 0, -0.306891, -0, 1, 0, -0, 0.39893, -0, -0.707107, -0, 0.707107, 0.0245, 0, 0.472, 0, -1, 0, -0, 0.107, 0, 1, 0, 0, 0.107, 0, 0.088, 0, -1, 0, 0, 0, 0, -6.12323e-17, 0, -1 });
        real_jacobian[8] = std::array<double, 42>({ 0, 0.306891, 0, 0, 0, 1, 0.153882, 0, -0.306891, -0, 1, 0, 0, 0.325815, -0, -0.707107, -0, 0.707107, 0.1279, 0, 0.472, 0, -1, 0, -0, 0.2104, 0, 1, 0, 0, 0.2104, 0, 0.088, 0, -1, 0, 0, 1.54074e-33, 0, -6.12323e-17, 0, -1 });
        real_jacobian[9] = std::array<double, 42>({ 0, 0.306891, 0, 0, 0, 1, 0.153882, 0, -0.306891, -0, 1, 0, 0, 0.325815, -0, -0.707107, -0, 0.707107, 0.1279, 0, 0.472, 0, -1, 0, -0, 0.2104, 0, 1, 0, 0, 0.2104, 0, 0.088, 0, -1, 0, 0, 1.54074e-33, 0, -6.12323e-17, 0, -1 });
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        franka::Frame frame = franka::Frame::kJoint1;
        for (size_t i = 0; i < 10; i++, frame++) EXPECT_NO_THROW(real_jacobian[i] = real_model->zeroJacobian(frame, q, real_state.F_T_EE, real_state.EE_T_K));
    }
    #endif
    std::cout << "Real jacobians:" << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        array_print(real_jacobian[i]);
        std::cout << std::endl;
    }
    
    //Emulator jacobian
    std::array<double, 42> emulator_jacobian[10];
    FRANKA_EMULATOR::Frame frame = FRANKA_EMULATOR::Frame::kJoint1;
    for (size_t i = 0; i < 10; i++, frame++) EXPECT_NO_THROW(emulator_jacobian[i] = emulator_model->zeroJacobian(frame, q, emulator_state.F_T_EE, emulator_state.EE_T_K));
    std::cout << "Emulator jacobians:" << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        array_print(emulator_jacobian[i]);
        std::cout << std::endl;
    }

    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint1)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint1)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint2)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint2)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint3)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint3)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint4)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint4)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint5)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint5)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint6)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint6)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint7)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint7)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kFlange)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kFlange)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kEndEffector)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kEndEffector)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kStiffness)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kStiffness)]), absolute_tolerance);
}

TEST(Model, BodyJacobianAccuracy)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    //Real jacobian
    std::array<double, 42> real_jacobian[10];
    if (real_ip == "")
    {
        real_jacobian[0] = std::array<double, 42>({ 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[1] = std::array<double, 42>({ 0, 0, 0, 0.707107, -0.707107, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[2] = std::array<double, 42>({ -0, -0.223446, 0, 0.707107, -0, 0.707107, 0.316, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[3] = std::array<double, 42>({ -0, 0, 0.165109, -1, 0, 0, -0.165109, 0.281782, 0, 0, 0, -1, 0, 0, -0.0825, -0.707107, -0.707107, -0, 0, 0, 0, -0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[4] = std::array<double, 42>({ 0, 0.218891, 0, -1, 0, 0, 0.218891, -0, 0.364282, 0, 1, 0, 0, 0.412365, 0, -0.707107, 0, -0.707107, -0.384, 0, -0.0825, 0, -1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });
        real_jacobian[5] = std::array<double, 42>({ 0, 0, -0.218891, -6.12323e-17, 1, 0, 0.364282, -0.218891, 0, 0, -0, -1, 0, -0, -0.412365, -0.707107, 0.707107, 0, -0.0825, 0.384, 0, 0, 0, 1, 0, 0, 0, 1, 6.12323e-17, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 });
        real_jacobian[6] = std::array<double, 42>({ -0.217004, -0.217004, 0, -4.32978e-17, 4.32978e-17, -1, 0.257586, -0.257586, 0.306891, -0.707107, -0.707107, 0, -0.335586, -0.335586, 0, -0.5, 0.5, -0.707107, -0.0583363, 0.0583363, -0.472, 0.707107, 0.707107, -0, -3.81021e-18, -3.81021e-18, 0, 0.707107, -0.707107, -6.12323e-17, 0, -7.70372e-34, -0.088, 0.707107, 0.707107, 0, 0, 0, 0, 0, 0, 1 });
        real_jacobian[7] = std::array<double, 42>({ -0.217004, -0.217004, -0, -4.32978e-17, 4.32978e-17, -1, 0.181926, -0.181926, 0.306891, -0.707107, -0.707107, 0, -0.282086, -0.282086, 0, -0.5, 0.5, -0.707107, 0.0173241, -0.0173241, -0.472, 0.707107, 0.707107, -0, -0.0756604, -0.0756604, 0, 0.707107, -0.707107, -6.12323e-17, 0.0756604, -0.0756604, -0.088, 0.707107, 0.707107, -0, 0, 0, 0, 0, 0, 1 });
        real_jacobian[8] = std::array<double, 42>({ 1.70358e-17, -0.306888, 0, -6.12318e-17, -6.16298e-33, -1, 0.153881, 8.54217e-18, 0.306891, 5.55112e-17, -0.99999, 0, 1.80864e-17, -0.325812, 0, -0.7071, -3.92523e-17, -0.707107, 0.127899, 7.09988e-18, -0.472, -5.55112e-17, 0.99999, -0, 1.16795e-17, -0.210398, 0, 0.99999, 5.55112e-17, -6.12323e-17, 0.210398, 1.16795e-17, -0.088, -5.55112e-17, 0.99999, 0, 8.55285e-50, -1.54073e-33, 0, 0, 2.7639e-33, 1 });
        real_jacobian[9] = std::array<double, 42>({ 1.70358e-17, -0.306888, 0, -6.12318e-17, -6.16298e-33, -1, 0.153881, 8.54217e-18, 0.306891, 5.55112e-17, -0.99999, 0, 1.80864e-17, -0.325812, 0, -0.7071, -3.92523e-17, -0.707107, 0.127899, 7.09988e-18, -0.472, -5.55112e-17, 0.99999, -0, 1.16795e-17, -0.210398, 0, 0.99999, 5.55112e-17, -6.12323e-17, 0.210398, 1.16795e-17, -0.088, -5.55112e-17, 0.99999, 0, 8.55285e-50, -1.54073e-33, 0, 0, 2.7639e-33, 1 });
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        franka::Frame frame = franka::Frame::kJoint1;
        for (size_t i = 0; i < 10; i++, frame++) EXPECT_NO_THROW(real_jacobian[i] = real_model->bodyJacobian(frame, q, real_state.F_T_EE, real_state.EE_T_K));
    }
    #endif
    std::cout << "Real jacobians:" << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        array_print(real_jacobian[i]);
        std::cout << std::endl;
    }
    
    //Emulator jacobian
    std::array<double, 42> emulator_jacobian[10];
    FRANKA_EMULATOR::Frame frame = FRANKA_EMULATOR::Frame::kJoint1;
    for (size_t i = 0; i < 10; i++, frame++) EXPECT_NO_THROW(emulator_jacobian[i] = emulator_model->bodyJacobian(frame, q, emulator_state.F_T_EE, emulator_state.EE_T_K));
    std::cout << "Emulator jacobians:" << std::endl;
    for (size_t i = 0; i < 10; i++)
    {
        array_print(emulator_jacobian[i]);
        std::cout << std::endl;
    }

    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint1)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint1)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint2)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint2)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint3)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint3)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint4)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint4)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint5)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint5)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint6)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint6)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint7)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kJoint7)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kFlange)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kFlange)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kEndEffector)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kEndEffector)]), absolute_tolerance);
    EXPECT_LE(array_difference(real_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kStiffness)], emulator_jacobian[static_cast<int>(FRANKA_EMULATOR::Frame::kStiffness)]), absolute_tolerance);
}

TEST(Model, GravityTime)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    //Real gravity
    std::array<double, 7> real_gravity;
    timespec real_start, real_finish;
    if (real_ip == "")
    {
        real_start.tv_sec = 0;
        real_start.tv_nsec = 0;
        real_finish.tv_sec = 0;
        real_finish.tv_nsec = 62217; //Intel(R) Xeon(R) W-2123 CPU @ 3.60GHz
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        clock_gettime(CLOCK_MONOTONIC, &real_start);
        EXPECT_NO_THROW(real_gravity = real_model->gravity(q, 0.0, std::array<double, 3>({0,0,0})));
        clock_gettime(CLOCK_MONOTONIC, &real_finish);
    }
    #endif
    std::cout << "Real gravity computed in " << 0.001 * timespec_difference(real_start, real_finish) << "us" << std::endl;
    
    //Emulator gravity
    std::array<double, 7> emulator_gravity;
    timespec emulator_start, emulator_finish;
    clock_gettime(CLOCK_MONOTONIC, &emulator_start);
    EXPECT_NO_THROW(emulator_gravity = emulator_model->gravity(q, 0.0, std::array<double, 3>({0,0,0})));
    clock_gettime(CLOCK_MONOTONIC, &emulator_finish);
    std::cout << "Emulator gravity computed in " << 0.001 * timespec_difference(emulator_start, emulator_finish) << "us" << std::endl;

    EXPECT_LE(timespec_difference(emulator_start, emulator_finish), 1000*1000);
}

TEST(Model, GravityAccuracyWithoutEndEffector)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    
    //Real gravity
    std::array<double, 7> real_gravity;
    if (real_ip == "") real_gravity = { 0, -1.73553, -0.670751, 18.5229, 0.715627, 1.67152, 2.45449e-18 };
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else EXPECT_NO_THROW(real_gravity = real_model->gravity(q, 0.0, std::array<double, 3>({0,0,0})));
    #endif
    std::cout << "Real gravity:" << std::endl;
    array_print(real_gravity);
    std::cout << std::endl;
    
    //Emulator gravity
    std::array<double, 7> emulator_gravity;
    EXPECT_NO_THROW(emulator_gravity = emulator_model->gravity(q, 0.0, std::array<double, 3>({0,0,0})));
    std::cout << "Emulator gravity:" << std::endl;
    array_print(emulator_gravity);
    std::cout << std::endl;
    
    EXPECT_LE(array_difference(real_gravity, emulator_gravity), absolute_tolerance);
}

TEST(Model, GravityAccuracyWithEndEffector)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    
    //Real gravity
    std::array<double, 7> real_gravity;
    if (real_ip == "") real_gravity = { 0, -3.93327, -0.670751, 21.903, 0.715627, 2.30172, 2.45449e-18 };
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else EXPECT_NO_THROW(real_gravity = real_model->gravity(q, real_state.m_total, std::array<double, 3>({0,0,0})));
    #endif
    std::cout << "Real gravity:" << std::endl;
    array_print(real_gravity);
    std::cout << std::endl;
    
    //Emulator gravity
    std::array<double, 7> emulator_gravity;
    EXPECT_NO_THROW(emulator_gravity = emulator_model->gravity(q, emulator_state.m_total, std::array<double, 3>({0,0,0})));
    std::cout << "Emulator gravity:" << std::endl;
    array_print(emulator_gravity);
    std::cout << std::endl;
    
    EXPECT_LE(array_difference(real_gravity, emulator_gravity), absolute_tolerance);
}

TEST(Model, CoriolisTime)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    std::array<double, 7> dq = { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7 };
    
    //Real coriolis
    std::array<double, 7> real_coriolis;
    timespec real_start, real_finish;
    if (real_ip == "")
    {
        real_start.tv_sec = 0;
        real_start.tv_nsec = 0;
        real_finish.tv_sec = 0;
        real_finish.tv_nsec = 18203; //Intel(R) Xeon(R) W-2123 CPU @ 3.60GHz
    }
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else
    {
        clock_gettime(CLOCK_MONOTONIC, &real_start);
        EXPECT_NO_THROW(real_coriolis = real_model->coriolis(q, dq, std::array<double, 9>({0,0,0,0,0,0,0,0,0}), 0.0, std::array<double, 3>({0,0,0})));
        clock_gettime(CLOCK_MONOTONIC, &real_finish);
    }
    #endif
    std::cout << "Real coriolis computed in " << 0.001 * timespec_difference(real_start, real_finish) << "us" << std::endl;
    
    //Emulator coriolis
    std::array<double, 7> emulator_coriolis;
    timespec emulator_start, emulator_finish;
    clock_gettime(CLOCK_MONOTONIC, &emulator_start);
    EXPECT_NO_THROW(emulator_coriolis = emulator_model->coriolis(q, dq, std::array<double, 9>({0,0,0,0,0,0,0,0,0}), 0.0, std::array<double, 3>({0,0,0})));
    clock_gettime(CLOCK_MONOTONIC, &emulator_finish);
    std::cout << "Emulator coriolis computed in " << 0.001 * timespec_difference(emulator_start, emulator_finish) << "us" << std::endl;

    EXPECT_LE(timespec_difference(emulator_start, emulator_finish), 1000*1000);
}

TEST(Model, CoriolisAccuracyWithoutEndEffector)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    std::array<double, 7> dq = { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7 };

    //Real coriolis
    std::array<double, 7> real_coriolis;
    if (real_ip == "") real_coriolis = { -0.0530698, -0.126959, -0.107617, -0.0264719, -0.00944426, -0.0130887, -0.000418606 };
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else EXPECT_NO_THROW(real_coriolis = real_model->coriolis(q, dq, std::array<double, 9>({0,0,0,0,0,0,0,0,0}), 0.0, std::array<double, 3>({0,0,0})));
    #endif
    std::cout << "Real coriolis:" << std::endl;
    array_print(real_coriolis);
    std::cout << std::endl;

    //Emulator coriolis
    std::array<double, 7> emulator_coriolis;
    EXPECT_NO_THROW(emulator_coriolis = emulator_model->coriolis(q, dq, std::array<double, 9>({0,0,0,0,0,0,0,0,0}), 0.0, std::array<double, 3>({0,0,0})));
    std::cout << "Emulator coriolis:" << std::endl;
    array_print(emulator_coriolis);
    std::cout << std::endl;

    EXPECT_LE(array_difference(real_coriolis, emulator_coriolis), absolute_tolerance);
}

TEST(Model, CoriolisAccuracyWithEndEffector)
{
    std::array<double, 7> q = { 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4 };
    std::array<double, 7> dq = { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7 };

    //Real coriolis
    std::array<double, 7> real_coriolis;
    if (real_ip == "") real_coriolis = { -0.0774557, -0.163096, -0.138512, -0.0174418, -0.0183765, -0.0222752, 0.000177797 };
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
    else EXPECT_NO_THROW(real_coriolis = real_model->coriolis(q, dq, real_state.I_total, real_state.m_total, std::array<double, 3>({0,0,0})));
    #endif
    std::cout << "Real coriolis:" << std::endl;
    array_print(real_coriolis);
    std::cout << std::endl;

    //Emulator coriolis
    std::array<double, 7> emulator_coriolis;
    EXPECT_NO_THROW(emulator_coriolis = emulator_model->coriolis(q, dq, emulator_state.I_total, emulator_state.m_total, std::array<double, 3>({0,0,0})));
    std::cout << "Emulator coriolis:" << std::endl;
    array_print(emulator_coriolis);
    std::cout << std::endl;

    EXPECT_LE(array_difference(real_coriolis, emulator_coriolis), absolute_tolerance);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    if (argc == 1) real_ip = "";    
    #if defined(Franka_FOUND) && !defined(FRANKA_EMULATOR_IMITATE)
        else if (argc == 2)
        {
            real_ip = argv[1];
            real_robot = new franka::Robot(real_ip);
            real_model = new franka::Model(real_robot->loadModel());
            real_state = real_robot->readOnce();
        }
        else
        {
            std::cout << "Usage:" << std::endl;
            std::cout << "./franka_emulator_selftest      - to test against recorded values" << std::endl;
            std::cout << "./franka_emulator_selftest <IP> - to test against libfranka" << std::endl;
        }
    #else
        else
        {
            std::cout << "Usage:" << std::endl;
            std::cout << "./franka_emulator_selftest      - to test against recorded values" << std::endl;
        }
    #endif
    
    FRANKA_EMULATOR::Network emulator_network;
    emulator_model = new FRANKA_EMULATOR::Model(emulator_network);
    emulator_state.m_total = FRANKA_EMULATOR::emulator::gripper_mass;
    emulator_state.I_total = FRANKA_EMULATOR::emulator::gripper_inertia;
    emulator_state.F_T_EE = FRANKA_EMULATOR::emulator::nominal_gripper_frame;
    emulator_state.EE_T_K = std::array<double, 16>({ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 });
    return RUN_ALL_TESTS();
}