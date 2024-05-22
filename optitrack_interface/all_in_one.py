# WIFI CONFIG
# SSID: OptiTrack
# PSW: ***********

import os
import rclpy
from rclpy.node import Node
from time import time
from math import atan2, asin
from optitrack_interface.nat_net_client import NatNetClient

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry

TRACKED_ROBOT_ID = 44 # IMPORTANT: must match the streaming ID of the optitrack program
if 'TRACKED_ROBOT_ID' in os.environ:
  TRACKED_ROBOT_ID=int(os.environ['TRACKED_ROBOT_ID'])
  print("found track id:",TRACKED_ROBOT_ID)
else:
  print("default track id:",TRACKED_ROBOT_ID)

class Optitrack(Node):
  def __init__(self):
    super().__init__('optitrack_node')
    # self.publisher_ = self.create_publisher(PoseStamped, '/optiTrack/pose', qos.qos_profile_sensor_data)

    self.calls = 0

    qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
    )

    # Create publishers
    self.odometry_publisher= self.create_publisher(
        VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', qos_profile)
    self.v_odometry_publisher= self.create_publisher(
        VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile)

    self.timer = self.create_timer(1.0, self.timer_callback)

    self.get_logger().info('Optitrack node running - version 2.1')

    #streamingClient = NatNetClient(ver=(3, 2, 0, 0), quiet=False)
    streamingClient = NatNetClient(ver=(3, 2, 0, 0), server_ip="192.168.1.33", quiet=False)
    streamingClient.rigidBodyListener = self.receiveRigidBodyFrame
    streamingClient.run()
    print("starting")

  def timer_callback(self):
    print("--------------------------------------------------")
    print("Hz: "+str(self.calls))
    
    self.calls = 0

  def receiveRigidBodyFrame(self, id, position, rotation):
    #print("recived body frame")
    if (id!=TRACKED_ROBOT_ID):
      print("found another object with id: ",id)
      print("current id: ",TRACKED_ROBOT_ID)
      return 0

    #ID matches
    msg = VehicleOdometry()
    msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
    msg.timestamp_sample = int(self.get_clock().now().nanoseconds / 1000)

    p = [position[0], position[2], -position[1]]
    msg.position=p
    
    q = [rotation[3],rotation[0], rotation[2], -rotation[1] ]
    msg.q=q
    msg.pose_frame=1 #NED frame
    msg.velocity_frame=1 #NED frame
    msg.quality = 1

    self.odometry_publisher.publish(msg)
    self.v_odometry_publisher.publish(msg)


    # self.publisher_.publish(msg)
    # self.calls = self.calls + 1

            

def main(args=None):
  rclpy.init(args=args)

  optitrack = Optitrack()

  rclpy.spin(optitrack)

  optitrack.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

