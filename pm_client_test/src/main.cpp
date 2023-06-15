#include "pm_client/client.hpp"

int main()
{
    PMClient::Client client;
    client.connect("opc.tcp://PC1M0484-1:4840");
    client.init();

    PMClient::Robot *robot = client.get_robot();

    std::cout << "X Position: " << robot->x_axis->get_position() << '\n';

    int r, g, b;
    robot->camera1->get_ring_light_color(r, g, b);
    std::cout << "Ring Light RGB: " << r << ' ' << g << ' ' << b << '\n';

    robot->camera1->set_ring_light_color(0, g, b);
}
