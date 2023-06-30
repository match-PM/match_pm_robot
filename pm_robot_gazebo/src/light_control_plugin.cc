#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Color.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <thread>
#include <memory>

namespace gazebo
{
    class LightController_ROS2_Plugin : public ModelPlugin
    {
        public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            
            // Initialize ROS 2 if it hasn't been already.
            if (!rclcpp::ok()) {
                rclcpp::init(0, nullptr);
            }

            this->rosNode = std::make_shared<rclcpp::Node>("gazebo_client");

            RCLCPP_INFO(this->rosNode->get_logger(), "ROS 2 Model Plugin Loaded!");

            this->sub_light = this->rosNode->create_subscription<std_msgs::msg::String>("/light_color", 1,
                std::bind(&LightController_ROS2_Plugin::ColourCallback, this, std::placeholders::_1));

            this->rosQueueThread = std::thread(
                std::bind(&LightController_ROS2_Plugin::QueueThread, this));

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&LightController_ROS2_Plugin::OnUpdate, this));

            gazebo::transport::NodePtr node(new gazebo::transport::Node());
            node->Init(model->GetWorld()->Name());
            this->light_pub = node->Advertise<gazebo::msgs::Light>("~/light/modify");
        }

        // ROS helper function that processes messages  
        void QueueThread() 
        {                                       
            rclcpp::spin(this->rosNode);
        }

        void control_light(std::string colour)
        {
            msgs::Light light_msg;

            light_msg.set_name(this->complete_light_name);

            if (colour == "red")  
            {
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(1.0, 0, 0.0, 1.0));    
            }

            if (colour == "green")  
            {
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(0.0, 1.0, 0.0, 1.0));
            }

            if (colour == "blue")  
            {
                msgs::Set(light_msg.mutable_diffuse(), ignition::math::Color(0.0, 0.0, 1.0, 1.0));
            }

            this->light_pub->Publish(light_msg);
        }

        void ColourCallback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->rosNode->get_logger(), "Received [%s]", msg->data.c_str());

            this->light_colour_name = msg->data;
        }

        void OnUpdate()
        {
            this->control_light(this->light_colour_name);
        }

        private:
        physics::ModelPtr model;
        std::string model_name, Link_name, light_name; 
        std::string complete_light_name;
        transport::PublisherPtr light_pub;
        event::ConnectionPtr updateConnection;
        std::shared_ptr<rclcpp::Node> rosNode;
        std::thread rosQueueThread;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_light;
        std::string light_colour_name = "red"; 
    };
    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LightController_ROS2_Plugin)
}
