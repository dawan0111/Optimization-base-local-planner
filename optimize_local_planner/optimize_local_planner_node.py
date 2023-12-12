import rclpy
import time
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

class OptimizeLocalPlannerNode(Node):
    def __init__(self):
        super().__init__('lidar_camera_projection_node')
        self.get_logger().info("===== OptimizeLocalPlannerNode =====")
        self.scan_subscriber_ = self.create_subscription(LaserScan, "/scan", self.scan_callback_, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, "/odom", self.odom_callback_, 10)
        self.timer_ = self.create_timer(0.05, self.control_callback_)
        self.scan_data_: LaserScan = None
        self.odom_data_: Odometry = None

        self.linear_velocity = -1.0
        self.angular_velocity = 0.0

        # world frame position
        self.goal = np.array([1, -0.5]).reshape(2, 1)
        self.dt = 0.05

    def control_callback_(self):
        if self.scan_data_ is None or self.odom_data_ is None:
            return
        self.get_cost(self.linear_velocity, self.angular_velocity)

    def get_cost(self, linear_velocity, angular_velocity):
        input_velocity = np.array([linear_velocity, angular_velocity]).reshape(2, 1)
        goal_cost = self.get_goal_cost(input_velocity) * 10

        self.get_logger().info("goal cost: {}".format(goal_cost))
        pass

    def get_goal_cost(self, input_velocity):
        x_k = self.robot_position[0]
        y_k = self.robot_position[1]
        yaw_k = self.robot_yaw

        A = np.array([
            x_k / input_velocity[0] + math.cos(yaw_k + input_velocity[1] * self.dt) * self.dt, 0,
            y_k / input_velocity[0] + math.sin(yaw_k + input_velocity[1] * self.dt) * self.dt, 0
        ]).reshape(2, 2)

        loss = self.goal - A @ input_velocity

        return loss.T @ loss
        
    def odom_callback_(self, odom_data: Odometry):
        self.odom_data_ = odom_data
        robot_pose = self.odom_data_.pose.pose

        position = np.array([robot_pose.position.x, robot_pose.position.y, robot_pose.position.z]).reshape(3, 1)
        quaternion = R.from_quat([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        
        self.robot_position = position
        self.robot_yaw = quaternion.as_euler("ZYX")[2]

    def scan_callback_(self, scan_data: LaserScan):
        angle_increment = scan_data.angle_increment
        polar_scan_data = []
        for i, r in enumerate(scan_data.ranges):
            polar_scan_data.append((i * angle_increment, r))

        self.scan_data_ = polar_scan_data

def main(args=None):
    rclpy.init(args=args)

    optimize_local_planner_node = OptimizeLocalPlannerNode()
    rclpy.spin(optimize_local_planner_node)
    optimize_local_planner_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
