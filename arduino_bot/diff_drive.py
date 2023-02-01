import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import Twist
import numpy as np
import math
import serial
l = 0.21
r=0.035   
class DiffDrive(Node):
    def __init__(self):
        super().__init__("diff_drive")
        self.vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.convert_to_vel, 10)
        self.ser = serial.Serial('/dev/ttyACM0', 57600)

    def convert_to_vel(self, msg: Twist):
        V_lin = msg.linear.x
        V_ang = msg.angular.z
        V_l = ((V_lin - (V_ang*l/2))/r)*60/(2*math.pi)
        V_r = ((V_lin + (V_ang*l/2))/r)*60/(2*math.pi)
        self.get_logger().info(f"{V_l} and {V_r}")
        self.write(V_l, V_r)

    def write(self, V_l, V_r): 
        self.ser.write(f'w{V_l} {V_r}\n'.encode('utf-8'))
        

 

def main(args=None):
    rclpy.init(args=args)
    node = DiffDrive()
    rclpy.spin(node)
    node.ser.close()
    rclpy.shutdown()

if __name__ == "__main__":
    main()