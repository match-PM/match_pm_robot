#include <cstdlib>
#include <iostream>

#include "pm_client/client.hpp"

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    PMClient::Client client;

    client.connect("opc.tcp://localhost:4840");
    auto &robot = client.get_robot();
    std::cout << "Axis X: \n"
              << "\tActual Position: " << robot.x_axis->get_position() << "\n"
              << "\tTarget Position: " << robot.x_axis->get_target() << "\n"
              << "\tTolerance: " << static_cast<int>(robot.x_axis->get_tolerance()) << "\n"
              << "\tSpeed: " << robot.x_axis->get_speed() << "\n";

    auto actual_position = robot.x_axis->get_position();
    auto target_position = actual_position + 100000.0;
    robot.x_axis->move(target_position);

    return EXIT_SUCCESS;
}

/*
 Pneumatik:
   Kleber 1k, 2k


*/
