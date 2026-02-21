# ============================================================
# IMPORTS
# ============================================================

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32

from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.marker_size = 0.20  # meters
        # simulated camara (should change in accordance to the actual camara parameters)
        #---------------------------------------
        self.camera_matrix = np.array([
            [600, 0, 320],
            [0, 600, 240],
            [0, 0, 1]
        ], dtype=np.float32)

        self.dist_coeffs = np.zeros((5, 1))
        #-------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters()

        # subscriptions

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        #publishers

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco/pose',
            10
        )

        self.detected_pub = self.create_publisher(
            Bool,
            '/aruco/detected',
            10
        )

        self.id_pub = self.create_publisher(
            Int32,
            '/aruco/id',
            10
        )

        self.get_logger().info("Aruco Detector Node Started")

    # Continous call back for a new image

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )

        if ids is not None:

            # 3d estimation of markers
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )

            tvec = tvecs[0][0]

            #publish results 

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()

            # Transformation values from cam to drone
            pose_msg.pose.position.x = float(tvec[2])
            pose_msg.pose.position.y = float(-tvec[0])
            pose_msg.pose.position.z = float(-tvec[1])

            self.pose_pub.publish(pose_msg)
            self.detected_pub.publish(Bool(data=True))
            self.id_pub.publish(Int32(data=int(ids[0])))

        else:
            self.detected_pub.publish(Bool(data=False))


#node activation

def main(args=None):

    
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()