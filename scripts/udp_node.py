#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from socket import *

def send_pose(data, args):
    server_name, server_port, client_socket = args[0], args[1], args[2]
    message = "{:f},{:f},{:f}".format(data.x,data.y,data.theta)
    client_socket.sendto(message.encode(), (server_name,server_port))
    print(message)

def main():
    server_name, server_port = '127.0.0.1', 5005
    client_socket = socket()
    client_socket = socket(AF_INET, SOCK_DGRAM)
    
    rospy.init_node('pose2D_sender', anonymous=True)
    rospy.Subscriber('/pose2D', Pose2D, send_pose, (server_name, server_port, client_socket))
    rospy.spin()
    return client_socket

if __name__ == '__main__':
    client_socket =  main()
    client_socket.close()
