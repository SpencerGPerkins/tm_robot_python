
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tm_msgs.srv import SetPositions

class SetPositionsClient(Node):
    def __init__(self):
        super().__init__('set_positions_client')
        self.client = self.create_client(SetPositions, 'set_positions')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.request = SetPositions.Request()

    def send_request(self):
        # Matching the TMSCT joint positions (converted to radians)
        self.request.motion_type = SetPositions.Request.PTP_J
        self.request.positions = [2.0, 0.1, 0.9, 0.0, 1.58, 0.0]  # Joint positions in radians
        self.request.velocity = 0.2  # 12% speed equivalent
        self.request.acc_time = 0.2  # 200ms acceleration time
        self.request.blend_percentage = 10
        self.request.fine_goal = False

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    client = SetPositionsClient()
    result = client.send_request()

    if result.ok:
        client.get_logger().info('OK')
    else:
        client.get_logger().info('Not OK')

    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from tm_msgs.srv import SetPositions


# class SetPositionsClient(Node):

#     def __init__(self):
#         super().__init__('demo_set_positions')
#         self.client = self.create_client(SetPositions, 'set_positions')

#         # Wait for the service to be available
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')

#     def send_request(self):
#         request = SetPositions.Request()
#         request.motion_type = SetPositions.Request.PTP_J
#         request.positions = [0., 0., 1.58, 0., 1.58, 0.]
#         request.velocity = 0.4  # rad/s
#         request.acc_time = 0.2
#         request.blend_percentage = 10
#         request.fine_goal = False

#         future = self.client.call_async(request)
#         rclpy.spin_until_future_complete(self, future)

#         if future.result() is not None:
#             if future.result().ok:
#                 self.get_logger().info('OK')
#             else:
#                 self.get_logger().info('Not OK')
#         else:
#             self.get_logger().error('Failed to call service')


# def main(args=None):
#     rclpy.init(args=args)

#     client = SetPositionsClient()
#     client.send_request()

#     client.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
