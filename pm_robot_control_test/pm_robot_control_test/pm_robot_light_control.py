import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool


## A service to controll the light in Gazebo. Does not work yet. 

class FlashlightController(Node):

    def __init__(self):
        super().__init__('flashlight_controller')

        self.flashlight_state = False

        self.srv_on = self.create_service(SetBool, 'flashlight/turn_on', self.turn_on_callback)
        self.srv_off = self.create_service(SetBool, 'flashlight/turn_off', self.turn_off_callback)

    def turn_on_callback(self, request, response):
        if not self.flashlight_state:
            self.flashlight_state = True
            # code that interacts with your Gazebo plugin
            response.success = True
            response.message = "Flashlight turned on."
        else:
            response.success = False
            response.message = "Flashlight is already on."
        return response

    def turn_off_callback(self, request, response):
        if self.flashlight_state:
            self.flashlight_state = False
            # code that interacts with your Gazebo plugin
            response.success = True
            response.message = "Flashlight turned off."
        else:
            response.success = False
            response.message = "Flashlight is already off."
        return response

def main(args=None):
    rclpy.init(args=args)

    flashlight_controller = FlashlightController()
    
    rclpy.spin(flashlight_controller)
    flashlight_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()