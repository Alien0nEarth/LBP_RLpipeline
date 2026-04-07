import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32MultiArray

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from px4_msgs.msg import BatteryStatus, VehicleStatus

from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion


class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')

        self.bridge = CvBridge()

        # ---------------- STATE ----------------
        self.state = {
            "pos": [0,0,0],
            "vel": [0,0,0],
            "att": [0,0,0],
            "battery": 0.0,
            "armed": 0,
            "mode": 0,

            "front": 10.0,
            "left": 10.0,
            "right": 10.0,
            "min_dist": 10.0,
            "collision": False,

            "depth_front": 10.0,
            "depth_left": 10.0,
            "depth_right": 10.0
        }

        # ---------------- SUBSCRIBERS ----------------

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.pos_cb,
            10)

        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.att_cb,
            10)

        self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status_v1',
            self.battery_cb,
            10)

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v2',
            self.status_cb,
            10)

        self.create_subscription(
            LaserScan,
            '/world/default/model/x500_depth_0/link/link/sensor/lidar_2d_v2/scan',
            self.lidar_cb,
            10)

        # OPTIONAL camera depth
        self.create_subscription(
            Image,
            '/camera/depth/image',
            self.depth_cb,
            10)

        # Publisher
        self.pub = self.create_publisher(Float32MultiArray, '/vehicle_state', 10)

        # Timer
        self.create_timer(0.05, self.publish_state)

    # ---------------- PX4 ----------------

    def pos_cb(self, msg):
        self.state["pos"] = [msg.x, msg.y, msg.z]
        self.state["vel"] = [msg.vx, msg.vy, msg.vz]

    def att_cb(self, msg):
        q = msg.q
        roll, pitch, yaw = euler_from_quaternion([q[1], q[2], q[3], q[0]])
        self.state["att"] = [roll, pitch, yaw]

    def battery_cb(self, msg):
        self.state["battery"] = msg.remaining

    def status_cb(self, msg):
        self.state["armed"] = msg.arming_state
        self.state["mode"] = msg.nav_state

    # ---------------- LIDAR ----------------

    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0)

        n = len(ranges)

        right = ranges[:n//3]
        front = ranges[n//3:2*n//3]
        left  = ranges[2*n//3:]

        self.state["front"] = float(np.min(front))
        self.state["left"]  = float(np.min(left))
        self.state["right"] = float(np.min(right))
        self.state["min_dist"] = float(np.min(ranges))
        self.state["collision"] = self.state["min_dist"] < 1.0

    # ---------------- DEPTH CAMERA ----------------

    def depth_cb(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        h, w = depth.shape

        center = depth[:, w//3:2*w//3]
        left = depth[:, :w//3]
        right = depth[:, 2*w//3:]

        self.state["depth_front"] = float(np.nanmean(center))
        self.state["depth_left"]  = float(np.nanmean(left))
        self.state["depth_right"] = float(np.nanmean(right))

    # ---------------- PUBLISH ----------------

    def publish_state(self):

        msg = Float32MultiArray()

        data = []

        data += self.state["pos"]
        data += self.state["vel"]
        data += self.state["att"]

        data += [self.state["battery"],
                 self.state["armed"],
                 self.state["mode"]]

        data += [self.state["front"],
                 self.state["left"],
                 self.state["right"],
                 self.state["min_dist"],
                 float(self.state["collision"])]

        data += [self.state["depth_front"],
                 self.state["depth_left"],
                 self.state["depth_right"]]

        msg.data = data

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
