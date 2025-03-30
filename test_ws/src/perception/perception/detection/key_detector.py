import os
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from paddleocr import PaddleOCR
import cv2
import os
import logging
from ppocr.utils.logging import get_logger
import pdb

@dataclass
class Homography:
    fx: float
    fy: float
    cx: float
    cy: float

    def convert_camera(self, cx: int, cy: int, depth: float) -> tuple[float, float]:
        """Converts given position of object in camera (cx,cy) with given depth to
        real X,Y position relative to the camera.
        """
        X = (cx - self.cx) / self.fx * depth
        Y = (cy - self.cy) / self.fy * depth
        return (float(X), float(Y))


class ObjectTFPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("object_detect_node")

        # Subscribe to image to run object detection on
        self.image_subscription = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )
        
        # Subscribe to depth to calculate the actual pose relative to camera
        self.depth_subscription = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_callback, 10
        )
        self.latest_depth_frame = None

        # Annotated image publisher with detections
        self.image_publisher = self.create_publisher(Image, "perception/object_tf/image", 10)

        # Model-specific parameters
        package_dir = get_package_share_directory("perception")
        self.valid_keys = self._read_in_keys_dict(os.path.join(package_dir, "model/keys_dict.txt"))

        # internal model
        self.bridge = CvBridge()
        self.model = PaddleOCR(use_angle_cls=True, lang='en')


        # TF magic
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to realsense intrinsics to get the homography matrix
        self.create_subscription(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.camera_info_callback, 10
        )
        self.homography: Homography | None = None

        # # Object Detection Params
        # self.declare_parameter(
        #     name="conf_threshold",
        #     descriptor=ParameterDescriptor(
        #         type=ParameterType.PARAMETER_DOUBLE,
        #         description="Obj detection model minimum confidence",
        #     ),
        #     value=0.2,
        # )

        # self.declare_parameter(
        #     name="iou_threshold",
        #     descriptor=ParameterDescriptor(
        #         type=ParameterType.PARAMETER_DOUBLE,
        #         description="Obj detection model IOU",
        #     ),
        #     value=0.3,
        # )

        # self.conf_thres = self.get_parameter("conf_threshold").get_parameter_value().double_value
        # self.iou_thres = self.get_parameter("iou_threshold").get_parameter_value().double_value
        # self.get_logger().info(f"starting node with {self.conf_thres=} {self.iou_thres=}")

        self.get_logger().info(f"starting node")

    def _read_in_keys_dict(self, path: str) -> None:
        """Reads in the keys dictionary from the given path and returns a list of keys."""
        with open(path, "r") as f:
            lines = f.readlines()
            lines = [line.strip() for line in lines]
            self.get_logger().info(f"read in keys dict: {lines}")
            return lines

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.homography = Homography(fx=msg.k[0], fy=msg.k[4], cx=msg.k[2], cy=msg.k[5])

    def calculate_depth(self, x1: int, y1: int, x2: int, y2: int) -> None | float:
        if self.latest_depth_frame is None:
            return None

        roi = self.latest_depth_frame[y1:y2, x1:x2]
        valid_depths = roi[~np.isnan(roi)]  # Remove NaN values

        # Convert from mm to meters
        return float(np.mean(valid_depths))/1000.0 if valid_depths.size > 0 else None

    def depth_callback(self, msg: Image) -> None:
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def image_callback(self, msg: Image) -> None:
        # self.get_logger().info("Received image")
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run OCR tracking on the frame

        # Testing inference time for model
        start_time = self.get_clock().now()
        results = self.model.ocr(rgb_frame, cls=True)
        end_time = self.get_clock().now()
        elapsed_time = (end_time - start_time)
        elapsed_time_seconds = elapsed_time.nanoseconds / 1e9
        self.get_logger().info(f"Elapsed time: {elapsed_time_seconds} seconds")

        self.get_logger().info(f"{results=}")

        # Process only the first result (change later for best detection)
        result = results[0] 
        if result is None or len(result) == 0:
            self.get_logger().error("no detection in result")
            return

        # Filter out results based on valid keys
        result = [
            [box, (text, confidence)]
            for box, (text, confidence) in result
            if text.upper() in self.valid_keys
        ]
        # [[[[451.0, 158.0], [636.0, 158.0], [636.0, 200.0], [451.0, 200.0]], ('contigo', 0.9973969459533691)]]

        # Iterate through the results and draw bounding boxes
        for line in result:
            self.get_logger().info(f"line: {line}")
            
            text: str = line[1][0]
            confidence: float = line[1][1]
            bbox = line[0]  # Bounding box coordinates

            x_min, y_min = int(bbox[0][0]), int(bbox[0][1])
            x_max, y_max = int(bbox[2][0]), int(bbox[2][1])

            # Draw bounding box
            cv2.rectangle(rgb_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(rgb_frame, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            x1, y1, x2, y2 = x_min, y_min, x_max, y_max
            cx, cy = int((x1 + x2) // 2), int((y1 + y2) // 2)

            depth = self.calculate_depth(x1, y1, x2, y2)
            if depth is None or np.isnan(depth):
                self.get_logger().error(f"no depth for key at {x1, y1, x2, y2}")
                continue

            if self.homography is None:
                self.get_logger().error("no homography info")
                return

            X, Y = self.homography.convert_camera(cx, cy, depth)

            # msg.header.frame_id
            self.publish_tf_global("camera_color_frame", (X, Y, depth), text)

        # Publish the annotated image   
        annotated_frame_image = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
        self.image_publisher.publish(annotated_frame_image)

    def publish_tf_global(
        self, parent_frame: str, object_pos: tuple[float, float, float], object_name: str
    ) -> None:
        # Notation: Z is realsense frame, O is object frame, I is utm frame.

        # 1. Get the transformation from Z to O (T_ZO)
        # Listen from tf buffer the transform from utm to realsense.

        # Change this to "rx200/base_link" if you want to use the base link of the robot as the static parent frame
        base_frame = "camera_link"
        self.get_logger().info(f"par frame: {parent_frame}")
        try:
            T_IZ_msg = self.tf_buffer.lookup_transform(base_frame, parent_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f"Could not transform rx200 baselink to Realsense: {ex}")
            return

        # 2. Convert the transform message into an actual transformation matrix from utm to realsense.
        T_IZ = np.eye(4, 4)
        T_IZ[0:3, 3] = np.array(
            [
                T_IZ_msg.transform.translation.x,
                T_IZ_msg.transform.translation.y,
                T_IZ_msg.transform.translation.z,
            ]
        )
        quaternion = [
            T_IZ_msg.transform.rotation.x,
            T_IZ_msg.transform.rotation.y,
            T_IZ_msg.transform.rotation.z,
            T_IZ_msg.transform.rotation.w,
        ]
        R_IZ = Rotation.from_quat(quaternion).as_matrix()
        T_IZ[0:3, 0:3] = R_IZ

        # 3. compute TF from realsense to object
        T_ZO = np.eye(4, 4)
        T_ZO[0:3, 3] = np.array(object_pos)  # position in realsense frame

        # 4. Compute the transform from utm to object
        T_IO = T_IZ @ T_ZO
        t_vec = T_IO[0:3, 3]
        quaternion = Rotation.from_matrix(T_IO[0:3, 0:3]).as_quat()

        self.get_logger().info(f"object {object_name} at {t_vec}")
        # 5. broadcast the TF of the object
        # Convert the transformation matrix to message and publish.
        new_msg = TransformStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = base_frame
        new_msg.child_frame_id = f"object_{object_name}"
        new_msg.transform.translation.x = t_vec[2]
        new_msg.transform.translation.y = -t_vec[0]
        new_msg.transform.translation.z = -t_vec[1]
        new_msg.transform.rotation.x = quaternion[0]
        new_msg.transform.rotation.y = quaternion[1]
        new_msg.transform.rotation.z = quaternion[2]
        new_msg.transform.rotation.w = quaternion[3]

        # Send the transformation
        self.tf_static_broadcaster.sendTransform(new_msg)


def main(args: Optional[list[str]] = None) -> None:
    # Ignore paddleocr debug warnings
    logger = get_logger()
    logger.setLevel(logging.ERROR)

    rclpy.init(args=args)
    object_tf_node = ObjectTFPublisherNode()
    rclpy.spin(object_tf_node)
    object_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
