# import rclpy

# from rclpy.node import Node
# from std_msgs.msg import String


# class nextKeyPush(Node):
#     def __init__(self):
#         super().__init__("next_key_listener")
#         self.subscription = self.create_subscription(
#             String, "next_arm_instruction", self.listener_callback, 10
#         )
#         # self.get_logger().info("Listener started listening!")
#         self.subscription

#     def listener_callback(self, msg):
#         self.get_logger().info(f"This is the next key the robot has to push {msg.data}")


# def main(args=None):
#     rclpy.init(args=args)
#     next_key_node = nextKeyPush()
#     rclpy.spin(next_key_node)
#     rclpy.shutdown()
