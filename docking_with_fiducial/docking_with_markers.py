import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import numpy as np
from scipy.spatial.transform import Rotation as R
import time
import math
import yaml
import os 
from ament_index_python.packages import get_package_share_directory


# stop seeing markers, DONE - check if safe 
# creation of yaml file - have to save in package config file get pacage share direct, - DONE
#launch file (should have realsense and aruco and this node with parameters namespace) and read me 
#add namespace (launches service in same namespace)

class DockWithMarkers(Node):

    def __init__(self):
        super().__init__("docking_with_markers")
        namespace = self.get_namespace()
        self.cmd_vel_publisher_ = self.create_publisher(Twist, f"{namespace}/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(MarkerArray, f"{namespace}/marker_publisher/markers", self.marker_callback, 10)
        self.srv_dock = self.create_service(Trigger, 'docking_with_markers', self.dock_callback)
        self.srv_param = self.create_service(Trigger, 'get_docking_offsets', self.dock_offsets_callback)
        self.timer = self.create_timer(0.1, self.dock_execute)
        
        self.stage = 'orient'
        self.docking = False

        self.marker_id = None
        self.marker_x = None
        self.marker_y = None
        self.marker_z = None
        self.marker_rot = None
        self.time_marker_last = None
        self.x_offset = None
        self.z_offset = None

        self.package_name = 'docking_with_fiducial'  # Replace with your actual package name
        self.file_name = 'docking_offsets.yaml'
        self.file_path = self.set_file_path(self.file_name)

        self.get_logger().info("Docking with Markers Node Initialized!")


    def set_file_path(self, file_name):
        current_dir = os.path.dirname(os.path.realpath(__file__))
        config_directory = os.path.join(current_dir, 'config')
        os.makedirs(config_directory, exist_ok=True)
        return os.path.join(config_directory, file_name)

    def dock_offsets_callback(self, request, response):
        if self.marker_id == None:
            response.success = False
            response.message = "No markers detected!"
            return response

        #offsets = {'marker_id':self.marker_id, 'lateral_bias':self.marker_x, 'distance':self.marker_z, 'angle_bias': self.quat_to_degrees(self.marker_rot)}
        offsets = {'marker_id':self.marker_id, 'lateral_bias':self.marker_x, 'distance':self.marker_z}

        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                existing_offsets = yaml.safe_load(file) 
        else:
            existing_offsets = {}
        
        existing_offsets[self.marker_id] = offsets
        
        with open(self.file_path, 'w') as file:
            yaml.safe_dump(existing_offsets, file)
        
        response.success = True
        response.message = "Docking parameters recorded"
        self.get_logger().info(f"Docking Parameters for Marker {self.marker_id} Recorded!")
        #self.get_logger().info(f"The file path is: {self.file_path}")
        #self.get_logger().info(os.getcwd())
        
        return response

    def dock_callback(self, request, response):

        self.get_logger().info("Docking service called")

        if self.marker_id == None:
            self.get_logger().info("No markers detected")
            self.docking = False
            response.success = False
            response.message = "No markers detected!"
            return response

        if os.path.exists(self.file_path):
            with open(self.file_path, 'r') as file:
                offsets = yaml.safe_load(file)
                
            if self.marker_id in offsets:
                offsets = offsets[self.marker_id]
                self.x_offset = offsets['lateral_bias'] 
                self.z_offset = offsets['distance']
                response.success = True
                response.message = "Docking Started!"
                #self.marker_rot = offsets['angle_bias']
            else:
                response.success = False
                response.message = f"No offsets for this ID: {self.marker_id}"
                return response
        else:
            response.success = False
            response.message = f"No docking offsets file found at {self.file_path}"
            return response
        
        self.stage = 'orient'
        self.docking = True

        return response


    def marker_callback(self, pose: MarkerArray):
        marker = pose.markers[0]
        self.marker_x = marker.pose.pose.position.x
        self.marker_y = marker.pose.pose.position.y
        self.marker_z = marker.pose.pose.position.z
        self.marker_rot = marker.pose.pose.orientation
        self.time_marker_last = time.time()

        self.marker_id = marker.id

    def dock_execute(self):

        # lateral_bias = 0
        # distance_bias = 0.75
        # yaw_bias = 0

        #self.check_if_safe()

        if self.marker_id == None or self.docking == False:
            return None

        self.get_logger().info(f"Docking Stage {self.stage}")

        x_pos = self.marker_x
        y_pos = self.marker_y
        z_pos = self.marker_z
        rot = self.marker_rot
        lateral_offset = x_pos - self.x_offset
  
        cmd = Twist()

        # STAGE 1/4: FIRST ORIENT 
        if self.stage == 'orient':
            self.check_if_safe()
            cmd, fixed = self.fix_orientation(rot, cmd)
            if fixed:
                self.stage = 'lateral_offset'

        # STAGE 2/4: FIX LATERAL OFFSET 
        elif self.stage == 'lateral_offset':
            #lateral_offset = x_pos - self.x_offset
            self.get_logger().info(f"Lateral Offset: {lateral_offset}")

            if abs(lateral_offset) < 0.05:
                self.stage = 'final_orient'
                # time.sleep(2)
            else:
                self.turn_angle(math.pi / 2, 3)
                self.move_linear(lateral_offset, 3)
                self.turn_angle(-math.pi / 2, 3)
                self.stage = 'final_orient_prep'
                # time.sleep(2)

        # # STAGE 2.5/4: FINAL ORIENT PREP [not pretty, we know!]
        elif self.stage == 'final_orient_prep':
            self.stage = 'final_orient'

        # # STAGE 3/4: FINAL ORIENT
        elif self.stage == 'final_orient':
            self.check_if_safe()
            cmd, fixed = self.fix_orientation(rot, cmd)
            if fixed:
                #time.sleep(2)
                self.stage = 'backup'
                

        # # STAGE 4/4: BACKUP
        #insert part where if the robot can no longer see the marker it stops given the initial distance 
        elif self.stage == 'backup':
            if z_pos > (self.z_offset + 0.01):
                #time.sleep(0.5)
                self.check_if_safe()
                cmd.linear.x = -0.10
            else:
                cmd.linear.x = 0.0
                self.get_logger().info(f"Done backing up lateral offset: {lateral_offset}")
                self.stage = 'completed'
                self.docking = False

        self.cmd_vel_publisher_.publish(cmd)

    def fix_orientation(self, rot, cmd):
        success = False
        R_mat = R.from_quat([rot.x, rot.y, rot.z, rot.w])
        yaw = R_mat.as_euler('xyz', degrees=True)[1]

        if abs(yaw) < 0.3:
            success = True
            cmd.angular.z = 0.0
            self.get_logger().info(f"Orientation fixed. Yaw: {yaw}")
        else:
            self.get_logger().info(f"Fixing Orientation. Yaw: {yaw}")
            if yaw > 0:
                cmd.angular.z = -0.1
            else:
                cmd.angular.z = 0.1

        return cmd, success

    def turn_angle(self, angle, time_s):
        start_time = time.time()
        cmd = Twist()
        while time.time() - start_time < time_s:
            #self.check_if_safe()
            cmd.angular.z = angle / time_s
            self.cmd_vel_publisher_.publish(cmd)
        cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

    def move_linear(self, distance, time_s):
        start_time = time.time()
        cmd = Twist()
        while time.time() - start_time < time_s:
            #self.check_if_safe()
            cmd.linear.x = distance / time_s
            self.cmd_vel_publisher_.publish(cmd)
        cmd.linear.x = 0.0
        self.cmd_vel_publisher_.publish(cmd)

    def check_if_safe(self):
        current_time = time.time()
        if current_time - self.time_marker_last > 1.0:
            self.docking = False
            self.get_logger().info(f"Marker out of sight since: {current_time - self.time_marker_last}")
            
    # def quat_to_degrees(self, rot):
    #     R_mat = R.from_quat([rot.x, rot.y, rot.z, rot.w])
    #     yaw = R_mat.as_euler('xyz', degrees=True)[1]
    #     return yaw

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
    node = DockWithMarkers()
    rclpy.spin(node)
    rclpy.shutdown()
    print("Exiting!")

if __name__ == '__main__':
    main()
