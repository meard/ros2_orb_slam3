#!/usr/bin/python3

"""
Python node for subscribe gazebo image and parse it to ORB SLAM3 System .

Author: Meard
Date: 04/05/2024

Requirements
* Make sure to set path to your workspace in common.hpp
* Please read Readme.md for installation and usage instructions
"""

# Imports
import rclpy
import ament_index_python.packages
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time


class imagePublisher(Node):

    def __init__(self):
        super().__init__('orb_img_publisher')

        # global initilized variables
        self.img_data = Image()
        self.timestamp = Float64()
        self.send_config = True  # Set False once handshake is completed with the cpp node
        self.settings_name = 'RealSense_D435i'  # Set
        self.br = CvBridge()

        # ROS 2 Publishers, Subscribers
        self.pub_img_to_agent_name = self.create_publisher(
            Image, '/mono_py_driver/img_msg', 10)

        self.pub_timestep_to_agent_name = self.create_publisher(
            Float64, '/mono_py_driver/timestep_msg', 10)

        self.publish_exp_config_ = self.create_publisher(
            String, '/mono_py_driver/experiment_settings', 10)  # Publish configs to the ORB-SLAM3 C++ node

        self.subscribe_exp_ack_ = self.create_subscription(String,
                                                           '/mono_py_driver/exp_settings_ack',
                                                           self.ack_callback, 10)

        self.sub_gz_img_ = self.create_subscription(
            Image,
            '/camera',
            self. img_callback,
            10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.count = 0

    # ****************************************************************************************
    def ack_callback(self, msg):
        """
            Callback function for Handshake acknowledgement
        """
        if (msg.data == "ACK") and self.count == 0:     # if ack is received once, confirm handshake aknowledgement and prevent any further multiple handshaking triggers
            self.count += 1
            self.send_config = False
    # ****************************************************************************************

    # ****************************************************************************************
    def handshake_with_cpp_node(self):
        """
            Send and receive acknowledge of sent configuration settings
        """
        if (self.send_config == True):
            msg = String()
            msg.data = self.settings_name
            self.publish_exp_config_.publish(msg)
            time.sleep(0.01)
    # ****************************************************************************************

    # ****************************************************************************************
    def img_callback(self, msg):
        """
            Callback function to receive gazebo image
        """
        msg = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        msg = cv2.resize(msg, (640, 480),
                         interpolation=cv2.INTER_LINEAR)
        self.img_data = cv2.cvtColor(msg, cv2.COLOR_BGR2GRAY)

        self.timestamp.data = float(time.time_ns() / 1000)
    # ****************************************************************************************

    # ****************************************************************************************
    def timer_callback(self):
        """
            Main Function
        """
        if self.i == 0:
            # DEBUG
            print(f"-------------- Received parameters --------------------------\n")
            print(f"self.settings_name: {self.settings_name}")
            print()
            print()
            print(f"MonoDriver initialized, attempting handshake with CPP node")

        if self.send_config == True:
            if self.count == 0:
                self.handshake_with_cpp_node()

        elif self.send_config == False and self.count == 1:
            print(f"Got ACK")
            print(f"Handshake complete")
            self.count += 1

            time.sleep(5)

        elif self.send_config == False and self.count > 1:
            msg = self.br.cv2_to_imgmsg(self.img_data, encoding='passthrough')
            timestep_msg = Float64()
            timestep_msg = self.timestamp

            try:
                self.pub_timestep_to_agent_name.publish(timestep_msg)
                self.pub_img_to_agent_name.publish(msg)
            except CvBridgeError as e:
                print(e)

        self.i += 1
    # ****************************************************************************************


# # # # ========================================
# # # # ==========    Main
# # # # ========================================


def main(args=None):
    rclpy.init(args=args)

    forb = imagePublisher()
    rclpy.spin(forb)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    forb.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
