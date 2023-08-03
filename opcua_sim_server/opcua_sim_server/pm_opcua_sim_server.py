# import asyncio
# import logging

# from asyncua import ua, Server


# async def main():
#     logging.basicConfig(level=logging.INFO)
#     self.server = Server()
#     self.server.set_endpoint("opc.tcp://localhost:4840")
#     self.server.set_server_name("test self.server")
#     await self.server.init()
#     await self.server.import_xml("./nodeset.xml")

#     async with self.server:
#         while True:
#             await asyncio.sleep(1)


# if __name__ == "__main__":
#     asyncio.run(main())


import rclpy
from rclpy.node import Node
import asyncio
import logging

from asyncua import ua, Server

class OPCUAServerNode(Node):
    def __init__(self):
        super().__init__('opcua_server_node')
        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('OPC UA Server Node has been initialized')
        self.loop = asyncio.get_event_loop()
        self.server = Server()
        self.loop.create_task(self.main())

    async def main(self):
        logging.basicConfig(level=logging.INFO)
        self.server.set_endpoint("opc.tcp://localhost:4840")
        self.server.set_server_name("test server")
        self.get_logger().info('OPC UA Server Node has been started')
        await self.server.init()
        await self.server.import_xml("nodeset.xml")

        async with self.server:
            while rclpy.ok():
                await asyncio.sleep(1)

    def destroy_node(self):
        self.loop.run_until_complete(self.server.stop())
        super().destroy_node()

    def timer_callback(self):
        self.loop.run_until_complete(asyncio.sleep(0.1))

def main(args=None):
    rclpy.init(args=args)
    node = OPCUAServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
