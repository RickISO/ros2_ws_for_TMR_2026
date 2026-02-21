import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from px4_msgs.msg import VehicleLocalPosition
from gate_detection.states import MissionState
import time


class MissionStateMachine(Node):

    def __init__(self):
        super().__init__('mission_state_machine')

        self.state = MissionState.TAKEOFF

        self.detected = False
        self.error = 0.0
        self.area = 0.0

        self.pass_start = None
        self.pass_duration = 2.0

        self.takeoff_height = -2.0
        self.local_pos = VehicleLocalPosition()

        self.create_subscription(Bool, '/gate/detected',
                                 self.detect_cb, 10)

        self.create_subscription(Float32, '/gate/error',
                                 self.error_cb, 10)

        self.create_subscription(Float32, '/gate/area',
                                 self.area_cb, 10)

        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self.pos_cb, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.loop)


    def detect_cb(self, msg):
        self.detected = msg.data

    def error_cb(self, msg):
        self.error = msg.data

    def area_cb(self, msg):
        self.area = msg.data

    def pos_cb(self, msg):
        self.local_pos = msg


    def loop(self):

        twist = Twist()

        match self.state:

            case MissionState.TAKEOFF:

                if self.local_pos.z > self.takeoff_height:
                    twist.linear.z = -0.5
                    self.vel_pub.publish(twist)
                else:
                    self.state = MissionState.SEARCH


            case MissionState.SEARCH:

                if not self.detected:
                    twist.linear.y = 0.3
                    self.vel_pub.publish(twist)
                else:
                    self.state = MissionState.ALIGN


            case MissionState.ALIGN:

                if abs(self.error) > 0.05:
                    twist.linear.y = -0.6 * self.error
                    self.vel_pub.publish(twist)
                else:
                    self.state = MissionState.APPROACH


            case MissionState.APPROACH:

                if self.area < 8000:
                    twist.linear.x = 0.5
                    self.vel_pub.publish(twist)
                else:
                    self.pass_start = time.time()
                    self.state = MissionState.PASS


            case MissionState.PASS:

                twist.linear.x = 1.0
                self.vel_pub.publish(twist)

                if time.time() - self.pass_start > self.pass_duration:
                    self.state = MissionState.SEARCH


            case MissionState.DONE:
                pass


def main():
    rclpy.init()
    node = MissionStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()