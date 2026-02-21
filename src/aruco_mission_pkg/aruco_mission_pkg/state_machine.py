import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleLocalPosition
from aruco_mission_pkg.states import MissionState
import time


class MissionStateMachine(Node):

    def __init__(self):
        super().__init__('state_matchine')

        self.state = MissionState.TAKEOFF

        self.aruco_detected = False
        self.aruco_pose = None
        self.local_pos = VehicleLocalPosition()

        self.takeoff_height = -2.0

        self.create_subscription(Bool, '/aruco/detected',
                                 self.detect_callback, 10)

        self.create_subscription(PoseStamped, '/aruco/pose',
                                 self.pose_callback, 10)

        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self.local_pos_callback, 10)

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String,
                                              '/master/in/control_mode', 10)

        self.timer = self.create_timer(0.1, self.loop)

    
    # Callbacks
   

    def detect_callback(self, msg):
        self.aruco_detected = msg.data

    def pose_callback(self, msg):
        self.aruco_pose = msg

    def local_pos_callback(self, msg):
        self.local_pos = msg

    
    # State Loop
    

    def loop(self):

        twist = Twist()

        match self.state:

            
            case MissionState.TAKEOFF:

                self.mode_pub.publish(String(data="velocity"))

                if self.local_pos.z > self.takeoff_height:
                    twist.linear.z = -0.5
                    self.vel_pub.publish(twist)
                else:
                    self.get_logger().info("Takeoff complete")
                    self.state = MissionState.SEARCH

            
            case MissionState.SEARCH:

                if not self.aruco_detected:
                    twist.linear.y = 0.3
                    self.vel_pub.publish(twist)
                else:
                    self.get_logger().info("Marker found")
                    self.state = MissionState.ALIGN

            
            case MissionState.ALIGN:

                error = self.aruco_pose.pose.position.y

                if abs(error) > 0.1:
                    twist.linear.y = -0.6 * error
                    self.vel_pub.publish(twist)
                else:
                    self.get_logger().info("Aligned")
                    self.state = MissionState.APPROACH

            
            case MissionState.APPROACH:

                distance = self.aruco_pose.pose.position.x

                if distance > 0.8:
                    twist.linear.x = 0.5
                    self.vel_pub.publish(twist)
                else:
                    self.get_logger().info("Close enough")
                    self.state = MissionState.PASS

            
            case MissionState.PASS:

                twist.linear.x = 1.0
                self.vel_pub.publish(twist)
                time.sleep(1.5)

                self.get_logger().info("Passed marker")
                self.state = MissionState.SEARCH

            
            case MissionState.IDLE:
                pass
def main(args=None):
    rclpy.init(args=args)
    node = MissionStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()