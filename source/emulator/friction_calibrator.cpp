#include <franka/robot.h>
#include <franka/model.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include <iostream>

enum class State
{
    move_to_default1,
    increment_torque,
    move_to_default2,
    decrement_torque
};

const unsigned int move_to_default_time = 3000;
const unsigned int try_number = 5;
const double torque_granularity = 0.001;
const double movement_treshold = 1 * M_PI / 180;

unsigned int joint_counter = 0;
unsigned int try_counter = 0;
unsigned int iteration_counter = 0;
State state = State::move_to_default1;
double joint_angle;
double upper_torque[7][try_number];
double lower_torque[7][try_number];

franka::Torques control(const franka::RobotState &robot_state, franka::Duration)
{
    franka::Torques torques(std::array<double, 7>{});
    if (state == State::move_to_default1 || state == State::move_to_default2)
    {
        const double target[7] = { 0.0, -M_PI / 4, 0.0, -3 * M_PI / 4, 0.0, M_PI / 2, M_PI / 4 };
        const double stiffness[7] = { 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0 };
        const double damping[7] = { 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0 };
        for (unsigned int i = 0; i < 7; i++) torques.tau_J[i] = stiffness[i] * (target[i] - robot_state.q[i]) - damping[i] * robot_state.dq[i];

        if (iteration_counter >= move_to_default_time)
        {
            joint_angle = robot_state.q[joint_counter];
            state = ((state == State::move_to_default1) ? State::increment_torque : State::decrement_torque);
            iteration_counter = 0;
            try_counter = 0;
            if (joint_counter == 7) torques.motion_finished = true;
        }
        else iteration_counter++;
    }
    else
    {
        for (unsigned int i = 0; i < 7; i++) torques.tau_J[i] = 0.0;
        torques.tau_J[joint_counter] = ((state == State::increment_torque) ? 1.0 : -1.0) * torque_granularity * iteration_counter;

        if (abs(robot_state.q[joint_counter] - joint_angle) > movement_treshold)
        {
            if (state == State::increment_torque)
            {
                upper_torque[joint_counter][try_counter] += torque_granularity * (iteration_counter - 1);
                state = State::move_to_default1;
            }
            else
            {
                lower_torque[joint_counter][try_counter] += torque_granularity * (iteration_counter - 1);
                state = State::move_to_default2;
                if (try_counter == try_number) joint_counter++;
            }
            iteration_counter = 0;
            try_counter++;
        }
        else iteration_counter++;
    }
    return torques;
}

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./franka_emulator_friction_calibrator <IP>" << std::endl;
        return 1;
    }

    //Init robot
    franka::Robot real_robot(argv[1]);
    
    //Starting cycle
    real_robot.control(control);

    //Printing parameters
    std::cout << "Calibration finished" << std::endl;
    for (unsigned int joint = 0; joint < 7; joint++)
    {
        std::cout << "<joint name=\"panda_joint" << joint+1 << "\" type=\"revolute\">" << std::endl;
        double friction = 0.0;
        for (unsigned int tru = 0; tru < try_number; tru++) friction += (upper_torque[joint][tru] - lower_torque[joint][tru]);
        friction /= try_number;
        std::cout << "<dynamics D=\"1\" K=\"7000\" damping=\"0.003\" friction=\"" << friction << "\" mu_coulomb=\"0\" mu_viscous=\"16\"/>" << std::endl;
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