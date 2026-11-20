#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pm_sensor_controller/sensor_controller.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

TEST(LaserSnapshot, ReadsHardwareWithoutOpcuaSkillOrPendingRequest)
{
    rclcpp::init(0, nullptr);
    {
        pm_sensor_controller::PMSensorController controller;
        ASSERT_EQ(controller.init("laser_snapshot_test"), controller_interface::return_type::OK);
        controller.configure();

        std::array<double, 8> values{};
        values[0] = 125.0;
        const auto names = controller.state_interface_configuration().names;
        std::vector<hardware_interface::StateInterface> states;
        states.reserve(names.size());
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            const auto separator = names[i].find('/');
            states.emplace_back(names[i].substr(0, separator), names[i].substr(separator + 1), &values[i]);
        }
        double bias = 0.0;
        hardware_interface::CommandInterface command("Force", "Bias", &bias);
        std::vector<hardware_interface::LoanedCommandInterface> commands;
        commands.emplace_back(command);
        std::vector<hardware_interface::LoanedStateInterface> loans;
        for (auto &state : states) loans.emplace_back(state);
        controller.assign_interfaces(std::move(commands), std::move(loans));
        controller.get_node()->activate();

        auto caller = std::make_shared<rclcpp::Node>("laser_snapshot_caller");
        auto client = caller->create_client<pm_msgs::srv::LaserGetMeasurement>(
            "/laser_snapshot_test/Laser/GetMeasurement");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(controller.get_node()->get_node_base_interface());
        executor.add_node(caller);
        ASSERT_TRUE(client->wait_for_service(2s));

        auto read = [&]() {
            auto future = client->async_send_request(
                std::make_shared<pm_msgs::srv::LaserGetMeasurement::Request>());
            if (executor.spin_until_future_complete(future, 1s) != rclcpp::FutureReturnCode::SUCCESS)
            {
                ADD_FAILURE() << "Snapshot service did not respond within one second";
                return std::numeric_limits<double>::quiet_NaN();
            }
            return future.get()->measurement;
        };
        // Before the first controller update there is no valid snapshot.
        EXPECT_TRUE(std::isnan(read()));
        auto update = [&]() {
            EXPECT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration(0, 1000000)),
                controller_interface::return_type::OK);
        };
        update();
        EXPECT_DOUBLE_EQ(read(), 125.0);

        // Back-to-back requests share the snapshot, with no acquisition slot.
        auto first = client->async_send_request(
            std::make_shared<pm_msgs::srv::LaserGetMeasurement::Request>());
        auto second = client->async_send_request(
            std::make_shared<pm_msgs::srv::LaserGetMeasurement::Request>());
        ASSERT_EQ(executor.spin_until_future_complete(first, 1s), rclcpp::FutureReturnCode::SUCCESS);
        ASSERT_EQ(executor.spin_until_future_complete(second, 1s), rclcpp::FutureReturnCode::SUCCESS);
        EXPECT_DOUBLE_EQ(first.get()->measurement, 125.0);
        EXPECT_DOUBLE_EQ(second.get()->measurement, 125.0);

        values[0] = -25.0;
        update();
        EXPECT_DOUBLE_EQ(read(), -25.0);
        values[0] = std::numeric_limits<double>::quiet_NaN();
        update();
        EXPECT_TRUE(std::isnan(read()));
        values[0] = 10.0;
        update();
        controller.get_node()->deactivate();
        EXPECT_TRUE(std::isnan(read()));
        controller.release_interfaces();
    }
    rclcpp::shutdown();
}
