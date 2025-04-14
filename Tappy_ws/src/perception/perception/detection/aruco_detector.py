import cv2
import numpy as np
import tf_transformations
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener

from perception_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import CameraInfo, Image

ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50
MARKER_SIZE = 0.03


class ArucoDetector(Node):
    """
    Node that detects Aruco AR markers in images and publishes their IDs and poses.

    Subscriptions
    /camera/image_raw (sensor_msgs.msg.Image): Raw image data from the camera.
    /camera/camera_info (sensor_msgs.msg.CameraInfo): Camera info and calibration parameters.

    Published Topics
        /aruco_poses (geometry_msgs.msg.PoseArray):
        PoseArray of all detected marker poses (suitable for RViz visualization).

        /aruco_markers (autonomous_interfaces.msg.ArucoMarkers):
        ArucoMarkers message containing marker poses and IDs.

    Parameters
        marker_size (float): Size of the markers in meters (default .0625)
        camera_frame (str): Frame to use for the camera (default: "").
    """

    def __init__(self):
        super().__init__("aruco_detector")

        self.declare_parameter(
            name="image_topic",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use",
            ),
        )

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        self.get_logger().info(f"Image topic: {image_topic}")
        self.get_logger().info(f"Image info topic: {info_topic}")
        self.get_logger().info(f"Camera frame topic: {self.camera_frame}")

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        # Set up subscriptions
        self.info_sub = self.create_subscription(CameraInfo, info_topic, self.info_callback, 10)
        self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Set up publishers
        self.poses_pub = self.create_publisher(PoseArray, "/aruco_poses", 10)
        self.markers_pub = self.create_publisher(ArucoMarkers, "/aruco_markers", 10)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        self.bridge = CvBridge()

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        # Unsubscribe from camera info to avoid repeated updates
        self.destroy_subscription(self.info_sub)

    def publish_tf(self, id, pose):
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        R_ZA = Rotation.from_quat(quaternion).as_matrix()
        t_ZA = np.array([pose.position.x, pose.position.y, pose.position.z]).reshape(3, 1)

        T_ZA = np.hstack((R_ZA, t_ZA))
        T_ZA = np.vstack((T_ZA, np.array([0, 0, 0, 1])))

        # Listen from tf buffer the transform from utm to zed.
        try:
            T_IZ_msg = self.tf_buffer.lookup_transform("rx200/base_link", self.camera_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().error(f"Could not transform utm to {self.camera_frame}: {ex}")
            return

        # Convert the transform message into an actual transformation matrix from utm to zed.
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

        # Compute the transform from utm to aruco (T_IA)
        T_IA = T_IZ @ T_ZA
        t_vec = T_IA[0:3, 3]
        quaternion = Rotation.from_matrix(T_IA[0:3, 0:3]).as_quat()

        # Convert the transformation matrix to message and publish.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "rx200/base_link"
        t.child_frame_id = f"aruco_id_{id}"
        t.transform.translation.x = t_vec[0]
        t.transform.translation.y = t_vec[1]
        t.transform.translation.z = t_vec[2]
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Send the transformation
        self.tf_static_broadcaster.sendTransform(t)

    def image_callback(self, img_msg):
        if self.info_msg is None:
            self.get_logger().warn("No camera info has been received!")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="mono8")

        # Prepare PoseArray and ArucoMarkers messages
        markers = ArucoMarkers()
        pose_array = PoseArray()

        if self.camera_frame == "":
            markers.header.frame_id = self.info_msg.header.frame_id
            pose_array.header.frame_id = self.info_msg.header.frame_id
        else:
            markers.header.frame_id = self.camera_frame
            pose_array.header.frame_id = self.camera_frame

        markers.header.stamp = img_msg.header.stamp
        pose_array.header.stamp = img_msg.header.stamp

        # Detect the markers in the image
        corners, marker_ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dictionary, parameters=self.aruco_parameters
        )

        # Process if a marker is detected
        if marker_ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, self.intrinsic_mat, self.distortion
            )

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_SIZE, self.intrinsic_mat, self.distortion
            )

            for i, marker_id in enumerate(marker_ids):
                pose = Pose()
                pose.position.x = tvecs[i][0][0]
                pose.position.y = tvecs[i][0][1]
                pose.position.z = tvecs[i][0][2]

                # Convert rotation vectors to quaternions
                rot_matrix = np.eye(4)
                rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                quat = tf_transformations.quaternion_from_matrix(rot_matrix)

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

                pose_array.poses.append(pose)
                markers.poses.append(pose)
                markers.marker_ids.append(marker_id[0])

                self.publish_tf(marker_id[0], pose)

            self.poses_pub.publish(pose_array)
            self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()
