#include "../include/franka_emulator/robot.h"
#include "../include/franka_emulator/gripper.h"
#include "../include/franka_emulator/model.h"
#include <iostream>

int _main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cout << "Usage: ./franka_emulator_example <IP>" << std::endl;
        return 1;
    }

    try
    {
        //Gripper example
        FRANKA_EMULATOR::Gripper gripper(argv[1]);
        gripper.move(0.1, 0.01);
        gripper.grasp(0.0, 0.01, 1.0);

        //Robot example
        FRANKA_EMULATOR::Robot robot(argv[1]);
        FRANKA_EMULATOR::Model model = robot.loadModel();
        size_t call = 0;
        robot.control([&call, &model](const FRANKA_EMULATOR::RobotState& robot_state, FRANKA_EMULATOR::Duration) -> FRANKA_EMULATOR::Torques
        {
            static const size_t period = 2000;
            static const size_t duration = 10000;
            call++;
            double delta = sin(2.0 * M_PI * (call % period) / period);
            std::array<double, 7> target = { 0.0, -M_PI / 4, 0.0, -3 * M_PI / 4, 0.0 + delta, M_PI / 2, M_PI / 4 + delta };
            std::array<double, 7> coriolis = model.coriolis(robot_state);
            static const double stiffness[7] = { 600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0 };
            static const double damping[7] = { 50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0 };
            FRANKA_EMULATOR::Torques torques(std::array<double, 7>{});
            for (size_t i = 0; i < 7; i++) torques.tau_J[i] = stiffness[i] * (target[i] - robot_state.q[i]) - damping[i] * robot_state.dq[i] + coriolis[i];
            torques.motion_finished = call > duration;
            return torques;
        });
    }
    catch (std::exception &e)
    {
        std::cout << "Exception occured: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    return _main(argc, argv);
}