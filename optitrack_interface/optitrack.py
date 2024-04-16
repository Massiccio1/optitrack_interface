# WIFI CONFIG
# SSID: OptiTrack
# PSW: ***********

import os
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from rclpy import qos
import socket
import struct
import numpy
from threading import Thread
from time import time
from math import atan2, asin
from optitrack_interface.nat_net_client import NatNetClient

TRACKED_ROBOT_ID = 44 # IMPORTANT: must match the streaming ID of the optitrack program
if 'TRACKED_ROBOT_ID' in os.environ:
  TRACKED_ROBOT_ID=int(os.environ['TRACKED_ROBOT_ID'])
  print("found track id:",TRACKED_ROBOT_ID)
else:
  print("default track id:",TRACKED_ROBOT_ID)

class Optitrack(Node):
  def __init__(self):
    super().__init__('optitrack_node')
    self.publisher_ = self.create_publisher(PoseStamped, '/optiTrack/pose', qos.qos_profile_sensor_data)

    self.calls = 0
    self.total=0

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
    print("total messages Hz: "+str(self.total))
    
    self.calls = 0
    self.total=0

  def receiveRigidBodyFrame(self, id, position, rotation):
    #print("recived body frame")
    self.total+=1
    if (id==TRACKED_ROBOT_ID):
    #if True:
      msg = PoseStamped()
      # retrieve as originally published by the optitrack
      msg.header.frame_id = "tag"
      msg.header.stamp = self.get_clock().now().to_msg()

      #publish as is, other pgograms will change reference system

      msg.pose.position.x = position[0]
      msg.pose.position.y = position[1]
      msg.pose.position.z = position[2]

      msg.pose.orientation.w = rotation[3]
      msg.pose.orientation.x = rotation[0] 
      msg.pose.orientation.y = rotation[1] 
      msg.pose.orientation.z = rotation[2]
      
      #msg.pose.orientation.w = 0.0
      #msg.pose.orientation.x = 0.0
      #msg.pose.orientation.y = 0.0
      #msg.pose.orientation.z = 0.0

      self.publisher_.publish(msg)
      self.calls = self.calls + 1
    else:
      print("found another object with id: ",id)
      print("current id: ",TRACKED_ROBOT_ID)
            

def main(args=None):
  rclpy.init(args=args)

  optitrack = Optitrack()

  rclpy.spin(optitrack)

  optitrack.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
