#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "pm_msgs/action/smaract_move.hpp"

class SmaractGripperActionServer : public rclcpp::Node
{
  public:
    using SmaractMove = pm_msgs::action::SmaractMove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<SmaractMove>;

  private:
    rclcpp_action::Server<SmaractMove>::SharedPtr m_move_server;

  public:
    explicit SmaractGripperActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("smaract_gripper_action_server", options)
    {
        m_move_server = rclcpp_action::create_server<SmaractMove>(
            this,
            "move",
            [this](auto uuid, auto goal) { return this->handle_goal(uuid, goal); },
            [this](auto goal_handle) { return this->handle_cancel(goal_handle); },
            [this](auto goal_handle) { return this->handle_accepted(goal_handle); }
        );
    }

  private:
    rclcpp_action::GoalResponse
    handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SmaractMove::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{[this](auto goal_handle) { return this->execute(goal_handle); }, goal_handle}
            .detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<SmaractMove::Result>();
        result->final_position = 999.9;
        goal_handle->succeed(result);
        // rclcpp::Rate loop_rate(1);
        // const auto goal = goal_handle->get_goal();
        // auto feedback = std::make_shared<SmaractMove::Feedback>();
        // auto &sequence = feedback->partial_sequence;
        // sequence.push_back(0);
        // sequence.push_back(1);
        // auto result = std::make_shared<SmaractMove::Result>();

        // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
        // {
        //     // Check if there is a cancel request
        //     if (goal_handle->is_canceling())
        //     {
        //         result->sequence = sequence;
        //         goal_handle->canceled(result);
        //         RCLCPP_INFO(this->get_logger(), "Goal canceled");
        //         return;
        //     }
        //     // Update sequence
        //     sequence.push_back(sequence[i] + sequence[i - 1]);
        //     // Publish feedback
        //     goal_handle->publish_feedback(feedback);
        //     RCLCPP_INFO(this->get_logger(), "Publish feedback");

        //     loop_rate.sleep();
        // }

        // // Check if goal is done
        // if (rclcpp::ok())
        // {
        //     result->sequence = sequence;
        //     goal_handle->succeed(result);
        //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        // }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(SmaractGripperActionServer)
