import argparse
import sys

import rclpy
from rclpy.node import Node

from pm_msgs.srv import GripperGetPosition, GripperGetVel, GripperMove, GripperMoveRel, GripperSetVel


class GripperServiceClient(Node):
    def __init__(self, service_namespace: str):
        super().__init__('gripper_service_client')

        namespace = service_namespace.strip() or '/pm_parallel_gripper_jaw_controller'
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        namespace = namespace.rstrip('/')

        self._move_client = self.create_client(GripperMove, f'{namespace}/Move')
        self._move_rel_client = self.create_client(GripperMoveRel, f'{namespace}/MoveRel')
        self._get_position_client = self.create_client(GripperGetPosition, f'{namespace}/GetPosition')
        self._set_vel_client = self.create_client(GripperSetVel, f'{namespace}/SetVel')
        self._get_vel_client = self.create_client(GripperGetVel, f'{namespace}/GetVel')

    def _call(self, client, request, timeout_sec: float = 3.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"Service '{client.srv_name}' not available")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done() or future.result() is None:
            raise RuntimeError(f"Service '{client.srv_name}' did not return a response")

        return future.result()

    def move(self, target_position: float):
        request = GripperMove.Request()
        request.target_position = float(target_position)
        response = self._call(self._move_client, request)
        print(f'success={response.success} error_msg={response.error_msg}')
        return 0 if response.success else 1

    def move_rel(self, offset: float):
        request = GripperMoveRel.Request()
        request.offset = float(offset)
        response = self._call(self._move_rel_client, request)
        print(f'success={response.success} error_msg={response.error_msg}')
        return 0 if response.success else 1

    def get_position(self):
        response = self._call(self._get_position_client, GripperGetPosition.Request())
        print(f'success={response.success} position={response.position}')
        return 0 if response.success else 1

    def set_vel(self, target_velocity: float):
        request = GripperSetVel.Request()
        request.target_velocity = float(target_velocity)
        response = self._call(self._set_vel_client, request)
        print(f'success={response.success}')
        return 0 if response.success else 1

    def get_vel(self):
        response = self._call(self._get_vel_client, GripperGetVel.Request())
        print(f'current_velocity={response.current_velocity}')
        return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description='Call the Unity parallel gripper services.')
    parser.add_argument('--service-namespace', default='/pm_parallel_gripper_jaw_controller',
                        help='Base namespace of the gripper services')

    subparsers = parser.add_subparsers(dest='command', required=True)

    move = subparsers.add_parser('move', help='Move the gripper to an absolute position in meters')
    move.add_argument('value', type=float)

    move_rel = subparsers.add_parser('move_rel', help='Move the gripper by a relative offset in meters')
    move_rel.add_argument('value', type=float)

    subparsers.add_parser('get_position', help='Read the current gripper position')

    set_vel = subparsers.add_parser('set_vel', help='Set the gripper target velocity in m/s')
    set_vel.add_argument('value', type=float)

    subparsers.add_parser('get_vel', help='Read the current gripper velocity')

    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init(args=None)
    node = GripperServiceClient(args.service_namespace)

    try:
        if args.command == 'move':
            return node.move(args.value)
        if args.command == 'move_rel':
            return node.move_rel(args.value)
        if args.command == 'get_position':
            return node.get_position()
        if args.command == 'set_vel':
            return node.set_vel(args.value)
        if args.command == 'get_vel':
            return node.get_vel()

        raise RuntimeError(f'Unknown command: {args.command}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
