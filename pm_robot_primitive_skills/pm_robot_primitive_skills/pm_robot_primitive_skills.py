from example_interfaces.srv import AddTwoInts
from pm_msgs.srv import DispenseForTime, EmptyWithSuccess
import rclpy
from rclpy.node import Node
import time

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import rclpy.time

class PrimitiveSkillsNode(Node):

    def __init__(self):
        super().__init__('pm_robot_primitive_skills')
        self.srv = self.create_service(DispenseForTime, self.get_name()+'/dispense_1K', self.dispense_callback)
        self.logger = self.get_logger()

        self.callback_group_re = ReentrantCallbackGroup()
        self.callback_group_mu_ex = MutuallyExclusiveCallbackGroup()

        self.client_dips_1k_on = self.create_client(EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/Pressure',callback_group = self.callback_group_re)
        self.client_dips_1k_off = self.create_client(EmptyWithSuccess,'/pm_nozzle_controller/Doseur_Nozzle/TurnOff',callback_group = self.callback_group_re)
        self.logger.info("Primitive skills node started!")

    def dispense_callback(self, request: DispenseForTime.Request, response:DispenseForTime.Response):
        
        self.logger.warn(f"{request.time}")

        if not self.client_dips_1k_on.wait_for_service(timeout_sec=1.0):
            self.logger.error("Client '/pm_nozzle_controller/Doseur_Nozzle/Pressure' not available!")
            response.success = False
            return response

        if not self.client_dips_1k_off.wait_for_service(timeout_sec=1.0):
            self.logger.error("Client '/pm_nozzle_controller/Doseur_Nozzle/TurnOff' not available!")
            response.success = False
            return response
        
        request_on = EmptyWithSuccess.Request()

        response_on:EmptyWithSuccess.Response = self.client_dips_1k_on.call(request_on)
        
        # future = self.client_dips_1k_on.call_async(request_on)

        # while not future.done():
        #     rclpy.spin_once(self)

        #response_on:EmptyWithSuccess.Response = future.result()

        if not response_on.success:
            response.success = False
            return response
        
        # This does not work yet --> find adtequate function for waiting
        self.create_rate(0.1)
        #time.sleep(request.time)

        request_off = EmptyWithSuccess.Request()

        response_off:EmptyWithSuccess.Response = self.client_dips_1k_off.call(request_off)

        # future_off = self.client_dips_1k_off.call_async(request_off)
        # while not future_off.done():
        #     rclpy.spin_once(self)
        # response_off:EmptyWithSuccess.Response = future_off.result()
        
        if not response_off.success:
            response.success = False
            return response    
        
        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)

    node = PrimitiveSkillsNode()

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
