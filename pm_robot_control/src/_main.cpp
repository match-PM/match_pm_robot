#undef NDEBUG

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "pm_client/client.hpp"
#include "pm_client/robot.hpp"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node
{
    PMClient::Client m_client;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_state_publisher;
    rclcpp::TimerBase::SharedPtr m_spin_timer;

  public:
    Controller() : Node("controller")
    {
        std::string endpoint{"opc.tcp://localhost:4840"};
        // std::string endpoint{"opc.tcp://PC1M0484-1:4840"};
        auto status = m_client.connect(endpoint);
        if (status != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "PMClient::connect: Verbindung fehlgeschlagen\n");
            return;
        }

        if (m_client.init() != 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "OPCUA SERVER BESITZT NICHT ALLE ERFORDERLICHEN KNOTEN"
            );
            return;
        }

        m_joint_state_publisher =
            this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        m_spin_timer = this->create_wall_timer(100ms, std::bind(&Controller::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        sensor_msgs::msg::JointState msg;

        msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::system_clock::now().time_since_epoch()
        )
                                   .count();

        auto robot = m_client.get_robot();
        msg.name.push_back("X_Axis_Joint");
        msg.position.push_back(
            robot->x_axis->increment_to_unit(robot->x_axis->get_position()) / 1e6
        );

        msg.name.push_back("Y_Axis_Joint");
        msg.position.push_back(
            robot->y_axis->increment_to_unit(robot->y_axis->get_position()) / 1e6
        );

        msg.name.push_back("Z_Axis_Joint");
        msg.position.push_back(
            robot->z_axis->increment_to_unit(robot->z_axis->get_position()) / 1e6
        );

        m_joint_state_publisher->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
