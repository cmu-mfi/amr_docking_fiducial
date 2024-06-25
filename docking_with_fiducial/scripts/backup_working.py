import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from docking.srv import DockingWithMarkers
import mathneobotix
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

class Backup(Node):

    def __init__(self):
        super().__init__("backup")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/robot1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(MarkerArray, "/marker_publisher/markers", self.pose_callback, 10)
        self.srv = self.create_service(DockingWithMarkers, 'docking_fiducial', self.handle_service)
        self.get_logger().info("Backup has started")
        self.stage = 'orient'
        self.is_oriented = False

    def handle_service(self, request, response):

        self.get_logger().info("Backup service called")
        self.stage = 'orient'
        self.is_oriented = False

        # Spin node until the backup is complete
        while self.stage != 'completed':
            rclpy.spin_once(self)

        response.success = True
        response.message = "Backup completed"
        return response


    def pose_callback(self, pose: MarkerArray):
        if len(pose.markers) == 0:
            self.get_logger().info("No markers detected")
            self.stage = 'orient'
            return

        marker = pose.markers[0]
        x_pos = marker.pose.pose.position.x
        y_pos = marker.pose.pose.position.y
        z_pos = marker.pose.pose.position.z
        rot = marker.pose.pose.orientation
        cmd = Twist()

        # STAGE 1/4: FIRST ORIENT 
        if self.stage == 'orient':
            cmd, fixed = self.fix_orientation(rot, cmd)
            if fixed:
                self.stage = 'lateral_offset'

        # STAGE 2/4: FIX LATERAL OFFSET 
        elif self.stage == 'lateral_offset':
            lateral_offset = x_pos
            self.get_logger().info(f"Lateral Offset: {lateral_offset}")

            if abs(lateral_offset) < 0.1:
                self.stage = 'final_orient'
                time.sleep(2)
            else:
                self.turn_angle(math.pi / 2, 3)
                self.move_linear(lateral_offset, 3)
                self.turn_angle(-math.pi / 2, 3)
                self.stage = 'final_orient'
                time.sleep(2)

        # # STAGE 3/4: FINAL ORIENT
        elif self.stage == 'final_orient':
            cmd, fixed = self.fix_orientation(rot, cmd)
            if fixed:
                time.sleep(2)
                self.stage = 'backup'
                

        # # STAGE 4/4: BACKUP
        elif self.stage == 'backup':
            if z_pos > 0.75:
                cmd.linear.x = -0.25
            else:
                cmd.linear.x = 0.0
                self.get_logger().info("Done backing up")
                self.stage = 'completed'

        self.cmd_vel_publisher_.publish(cmd)

    def fix_orientation(self, rot, cmd):
        success = False
        R_mat = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        yaw = R_mat.as_euler('xyz', degrees=True)[1]
        self.get_logger().info(f"Fixing Orientation. Yaw: {yaw}")

        if yaw > 0:
            cmd.angular.z = -0.1
        else:
            cmd.angular.z = 0.1

        if abs(yaw) < 0.5:
            self.get_logger().info("Fixed Orientation")
            success = True
            cmd.angular.z = 0.0
            self.get_logger().info("Orientation fixed")
        return cmd, success

    def turn_angle(self, angle, time_s):
        start_time = time.time()
        cmd = Twist()
        while time.time() - start_time < time_s:
            cmd.angular.z = angle / time_s
            self.cmd_vel_publisher_.publish(cmd)
        cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

    def move_linear(self, distance, time_s):
        start_time = time.time()
        cmd = Twist()
        while time.time() - start_time < time_s:
            cmd.linear.x = distance / time_s
            self.cmd_vel_publisher_.publish(cmd)
        cmd.linear.x = 0.0
        self.cmd_vel_publisher_.publish(cmd)

#   def quaternion_to_euler_degrees(self, quat):
#         rpy = [0, 0, 0]
#         rpy[0] = math.degrees(math.atan2(2.0 * (quat.w * quat.x + quat.y * quat.z), 1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)))
#         rpy[1] = math.degrees(math.asin(2.0 * (quat.w * quat.y - quat.z * quat.x)))
#         rpy[2] = math.degrees(math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)))
#         return tuple(rpy)
    
    
#   def quaternion_to_euler_radians(self, quat):
#         siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
#         cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
#         yaw = math.atan2(siny_cosp, cosy_cosp)
#         return yaw

#   def normalize_angle(self, angle):
#         while angle > 90.0 :
#             angle -= 180.0
#         while angle < -90.0:
#             angle += 180.0
#         return angle
    

def main(args=None):
    rclpy.init(args=args)
    node = Backup()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
