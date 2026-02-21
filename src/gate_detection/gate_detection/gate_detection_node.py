import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge


class GateDetection(Node):

    def __init__(self):
        super().__init__('gate_detection_node')

        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            '/camera_front',
            self.image_callback,
            10
        )

        self.detect_pub = self.create_publisher(Bool, '/gate/detected', 10)
        self.error_pub = self.create_publisher(Float32, '/gate/error', 10)
        self.area_pub = self.create_publisher(Float32, '/gate/area', 10)

        self.min_area_threshold = 500


    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        detected, error, area = self.gate_detect(frame)

        self.detect_pub.publish(Bool(data=detected))
        self.error_pub.publish(Float32(data=error))
        self.area_pub.publish(Float32(data=area))


    def gate_detect(self, cv_image):

        if cv_image is None:
            return False, 0.0, 0.0

        H, W, _ = cv_image.shape

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([40,50,50])
        upper_green = np.array([80,255,255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = np.ones((5,5), np.uint8)
        mask_eroded = cv2.erode(mask, kernel, iterations=1)
        mask_dilated = cv2.dilate(mask_eroded, kernel, iterations=1)
        blurred_mask = cv2.GaussianBlur(mask_dilated, (5,5), 0)

        contours, _ = cv2.findContours(
            blurred_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        image_center_x = W // 2

        for contour in contours:

            area = cv2.contourArea(contour)

            if area < self.min_area_threshold:
                continue

            x,y,w,h = cv2.boundingRect(contour)
            aspect_ratio = w/h if h != 0 else 0

            if 2.5 < aspect_ratio < 3.5:

                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                if 4 <= len(approx) <= 6:

                    M = cv2.moments(contour)

                    if M["m00"] != 0:
                        cx = int(M["m10"]/M["m00"])

                        error = cx - image_center_x

                        return True, float(error), float(area)

        return False, 0.0, 0.0


def main():
    rclpy.init()
    node = GateDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()