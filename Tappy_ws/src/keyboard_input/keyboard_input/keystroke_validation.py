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

        self.queue_list = []

        self.sentence = String()

        self.keyboard_subscription = self.create_subscription(
            String, "keyboard_inp", self.validate_keys, 10
        )

        self.instruction_publisher = self.create_publisher(
            String, "next_arm_instruction", 10
        )

        self.status_publisher = self.create_publisher(String, "type_status", 10)

        self.input_subscription = self.create_subscription(
            String, "Sentence_to_type", self.get_input_sentence, 10
        )

        self.get_logger().info(f"this is the list after population {self.queue_list}")

    def get_input_sentence(self, msg):
        self.sentence = msg.data
        self.populate_queue(self.sentence, self.queue_list)
        first_instr = String()
        first_instr.data = self.queue_list[0]
        self.instruction_publisher.publish(first_instr)

    def validate_keys(self, msg):
        actual_key = msg.data
        # predicted_key = self.queue_list[0]
        # self.get_logger().info(
        #     f"this is the key its on {predicted_key} and this is the key its typing {actual_key}"
        # )
        if len(self.queue_list) != 0:
            predicted_key = self.queue_list[0]
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
                elif actual_key == "backspace":
                    correct_char = self.queue_list[0]
                    self.queue_list.insert(0, correct_char)
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

        if self.queue_list:
            self.publish_next_instruction()

        else:
            self.publish_finish_status()
            # self.populate_queue(self.sentence, self.queue_list)

    def publish_next_instruction(self):
        next_instr = String()
        next_instr.data = self.queue_list[0]
        self.instruction_publisher.publish(next_instr)
        self.get_logger().info(f"published next instruction to arm {next_instr.data}")

    def populate_queue(self, sentence, queue_list):

        for c in sentence:
            if c.isspace():
                queue_list.append("space")
            elif c.isupper():
                queue_list.append("capslock")
                queue_list.append(c.lower())
                queue_list.append("capslock")
            # elif c == '/':
            #     queue_list.append("slash")
            else:
                queue_list.append(c)

    def publish_finish_status(self):
        finish_status = String()
        finish_status.data = "Done typing"
        self.status_publisher.publish(finish_status)
        self.get_logger().info("end of string reached")


def main(args=None):
    rclpy.init(args=args)
    validation_node = validateKeystrokes()
    rclpy.spin(validation_node)
    rclpy.shutdown()
