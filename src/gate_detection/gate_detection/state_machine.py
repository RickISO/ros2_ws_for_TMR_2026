#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from gate_detection.states import MissionState


class GateMissionStateMachine(Node):

    def __init__(self):

        super().__init__('gate_mission_state_machine')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Current state
        self.state = MissionState.IDLE

        # Var
        self.gate_detected = False
        self.gates_passed = 0
        self.max_gates = 5

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self.takeoff_height = -1.5
        self.start_alt = None

        # Subscribers
        self.create_subscription(
            Bool,
            '/gate/detected',
            self.gate_detect_callback,
            10
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos
        )

        # Publishers
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, '/master/in/mode', 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Gate Mission State Machine Ready")

    # Callbacks

    def gate_detect_callback(self, msg):
        self.gate_detected = msg.data

    def local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg


    # MAIN LOOP
    def timer_callback(self):

        if self.start_alt is None:
            self.start_alt = self.vehicle_local_position.z

        # TRANSITIONS
        # ----------------------------------------

        match self.state:

            case MissionState.IDLE:
                self.state = MissionState.TAKEOFF

            case MissionState.TAKEOFF:
                if self.vehicle_local_position.z <= self.takeoff_height:
                    self.get_logger().info("Takeoff complete")
                    self.state = MissionState.SEARCH_GATE

            case MissionState.SEARCH_GATE:
                if self.gate_detected:
                    self.get_logger().info("Gate detected")
                    self.state = MissionState.ALIGN_GATE

            case MissionState.ALIGN_GATE:
                # Aquí luego meteremos error real
                self.state = MissionState.APPROACH_GATE

            case MissionState.APPROACH_GATE:
                # Aquí luego meteremos área real
                self.state = MissionState.PASS_GATE

            case MissionState.PASS_GATE:
                self.gates_passed += 1
                self.get_logger().info(f"Gate passed: {self.gates_passed}")

                if self.gates_passed >= self.max_gates:
                    self.state = MissionState.FINISHED
                else:
                    self.state = MissionState.SEARCH_GATE

            case MissionState.FINISHED:
                pass


        # ACTIONS
        # ------------------------

        twist = Twist()

        match self.state:

            case MissionState.TAKEOFF:
                twist.linear.z = -0.5
                self.velocity_pub.publish(twist)

            case MissionState.SEARCH_GATE:
                twist.linear.y = 0.5
                self.velocity_pub.publish(twist)

            case MissionState.ALIGN_GATE:
                twist.linear.x = 0.2
                self.velocity_pub.publish(twist)

            case MissionState.APPROACH_GATE:
                twist.linear.x = 0.5
                self.velocity_pub.publish(twist)

            case MissionState.PASS_GATE:
                twist.linear.x = 1.0
                self.velocity_pub.publish(twist)
                time.sleep(1.2)

            case MissionState.FINISHED:
                self.mode_pub.publish(String(data="LAND"))


def main(args=None):
    rclpy.init(args=args)
    node = GateMissionStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()