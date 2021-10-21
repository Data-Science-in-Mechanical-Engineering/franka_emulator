#include "../include/franka/robot.h"
#include "../include/franka/model.h"
#include <iostream>

int _main()
{
    try
    {
        franka::Robot robot("192.168.0.1");
        franka::Model model = robot.loadModel();
        size_t call = 0;
        robot.control([&call, &model](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques
        {
            static const size_t period = 2000;
            static const size_t duration = 10000;
            call++;
            double delta = sin(2.0 * M_PI * (call % period) / period);
            std::array<double, 7> target = { 0.0, -M_PI / 4, 0.0, -3 * M_PI / 4 + delta, 0.0 + delta, M_PI / 2, M_PI / 4 + delta };
            std::array<double, 7> coriolis = model.coriolis(robot_state);
            static const std::array<double, 7> stiffness = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}};
            static const std::array<double, 7> damping = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};
            franka::Torques torques(std::array<double, 7>{});
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

int main()
{
    return _main();
}