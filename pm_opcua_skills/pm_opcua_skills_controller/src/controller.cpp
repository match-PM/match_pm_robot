#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "pm_opcua_skills_msgs/srv/dispense.hpp"
#include "pm_opcua_skills_msgs/srv/force_sensing_move.hpp"

#include "pm_client/client.hpp"

using namespace pm_opcua_skills_msgs::srv;

class Controller : public rclcpp::Node
{
    PMClient::Client m_client;

    rclcpp::Service<Dispense>::SharedPtr m_dispense_service;
    rclcpp::Service<ForceSensingMove>::SharedPtr m_force_sensing_move_service;

  public:
    Controller() : Node("pm_opcua_skills_controller")
    {
        m_client.connect("opc.tcp://PC1M0484-1:4840");
        m_client.init();

        m_dispense_service = create_service<Dispense>(
            "~/Dispense",
            [this](
                const Dispense::Request::SharedPtr request,
                Dispense::Response::SharedPtr response
            ) {
                const auto success = m_client.get_robot().skills->dispense(
                    request->time,
                    request->z_height,
                    request->z_move
                );
                response->success = success;
            }
        );

        m_force_sensing_move_service = create_service<ForceSensingMove>(
            "~/ForceSensingMove",
            [this](
                const ForceSensingMove::Request::SharedPtr request,
                ForceSensingMove::Response::SharedPtr response
            ) {
                const auto success = m_client.get_robot().skills->force_sensing_move(
                    request->start_x,
                    request->start_y,
                    request->start_z,
                    request->target_x,
                    request->target_y,
                    request->target_z,
                    request->max_fx,
                    request->max_fy,
                    request->max_fz,
                    request->step_size
                );
                response->success = success;
            }
        );

        RCLCPP_INFO(get_logger(), "running...");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
