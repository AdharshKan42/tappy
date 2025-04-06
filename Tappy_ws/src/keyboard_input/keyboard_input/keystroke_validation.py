# import rclpy

# from rclpy.node import Node
# from std_msgs.msg import String


# class validateKeystrokes(Node):
#     # example string for now, switch it to whatever the actual input string ends up being
#     pass
import rclpy

from rclpy.node import Node
from std_msgs.msg import String

# from keyboard_input.keystroke_listen import KeyboardListener
# from keyboard_input.keystroke_publish import KeyboardPublisher

# import queue


class validateKeystrokes(Node):
    # example string for now, switch it to whatever the actual input string ends up being
    def __init__(self):
        super().__init__("keyboard_validation")
        sentence = "I love to eat muffins on sundays/"
        self.subscription = self.create_subscription(
            String, "keyboard_inp", self.validate_keys, 10
        )
        self.queue_list = []
        self.populate_queue(sentence, self.queue_list)
        self.get_logger().info(f"this is the list after population {self.queue_list}")

    def validate_keys(self, msg):
        actual_key = msg.data
        predicted_key = self.queue_list[0]
        self.get_logger().info(
            f"this is the key its on {predicted_key} and this is the key its typing {actual_key}"
        )

        if actual_key == predicted_key:
            self.queue_list.pop(0)
        else:
            if (
                self.queue_list[0] == "capslock"
                and self.queue_list[2] == "capslock"
                and len(self.queue_list) >= 3
            ):
                correct_char = self.queue_list[1]
                self.queue_list = self.queue_list[3:]
                self.queue_list.insert(0, "capslock")
                self.queue_list.insert(0, correct_char)
                self.queue_list.insert(0, "capslock")
                self.queue_list.insert(0, "backspace")
                self.get_logger().info(
                    f"this is the list with new instructions added {self.queue_list}"
                )
            else:
                correct_char = self.queue_list[0]
                self.queue_list = self.queue_list[1:]
                self.queue_list.insert(0, correct_char)
                self.queue_list.insert(0, "backspace")
                self.get_logger().info(
                    f"this is the list with new instructions added {self.queue_list}"
                )

    def populate_queue(self, sentence, queue_list):

        for c in sentence:
            if c.isspace():
                queue_list.append("space")
            elif c.isupper():
                queue_list.append("capslock")
                queue_list.append(c.lower())
                queue_list.append("capslock")
            else:
                queue_list.append(c)


def main(args=None):
    rclpy.init(args=args)
    validation_node = validateKeystrokes()
    rclpy.spin(validation_node)
    rclpy.shutdown()
