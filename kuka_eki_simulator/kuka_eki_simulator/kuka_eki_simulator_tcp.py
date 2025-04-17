#!/usr/bin/env python3

import argparse
import sys
import socket
import numpy as np
import time
import xml.etree.ElementTree as ET
import errno
import rclpy
from std_msgs.msg import String

max_vel = 1.0 * 100.0

def create_eki_xml_rob(act_joint_pos):
    q = act_joint_pos
    qd = [0.0]*6
    eff = [0.0]*6

    root = ET.Element('RobotState')

    ET.SubElement(root, 'Pos', {'A1': str(q[0]), 'A2': str(q[1]), 'A3': str(q[2]),
                                'A4': str(q[3]), 'A5': str(q[4]), 'A6': str(q[5])})
    ET.SubElement(root, 'Vel', {'A1': str(qd[0]/max_vel), 'A2': str(qd[1]/max_vel), 'A3': str(qd[2]/max_vel),
                                'A4': str(qd[3]/max_vel), 'A5': str(qd[4]/max_vel), 'A6': str(qd[5]/max_vel)})
    ET.SubElement(root, 'Eff', {'A1': str(eff[0]), 'A2': str(eff[1]), 'A3': str(eff[2]),
                                'A4': str(eff[3]), 'A5': str(eff[4]), 'A6': str(eff[5])})
    ET.SubElement(root, 'RobotCommand', {'Size': str(1)})

    return ET.tostring(root, short_empty_elements=False)


def parse_eki_xml_sen(data):
    desired_joint = None

    try:
        root = ET.fromstring(data)
        if root:
            if root.tag == 'RobotCommand':
                pos = root.find('Pos').attrib
                if pos:
                    desired_joint = np.array([pos['A1'], pos['A2'], pos['A3'],
                                              pos['A4'], pos['A5'], pos['A6']], dtype=np.float64)
            else:
                print(f' RobotCommand not found in data {root}')

    except Exception as e:
        print(f' Cannot parse this data {data}, Error: {e}')
        pass

    return desired_joint


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='KUKA EKI Simulation over TCP')
    parser.add_argument('--eki_hw_iface_ip', default="127.0.0.1", help='The IP address of the EKI control interface (default=127.0.0.1)')
    parser.add_argument('--eki_hw_iface_port', default=54600, help='The port of the EKI control interface (default=54600)')
    parser.add_argument('--sen', default='ImFree', help='Type attribute in EKI XML doc. E.g. <Sen Type:"ImFree">')

    # Parse known arguments
    args, _ = parser.parse_known_args()
    host = args.eki_hw_iface_ip
    port = int(args.eki_hw_iface_port)
    sen_type = args.sen

    # Configuration
    node_name = 'kuka_eki_simulation_tcp'
    cycle_time = 0.004
    act_joint_pos = np.array([0, -90, 90, 0, 90, 0], dtype=np.float64)
    cmd_joint_pos = act_joint_pos.copy()
    des_joint_absolute = np.zeros(6)
    timeout_count = 0
    max_timeout = 5

    node = rclpy.create_node(node_name)

    node.get_logger().info(f"Started '{node_name}' node.")

    eki_act_pub = node.create_publisher(String, '~/eki/state', 1)
    eki_cmd_pub = node.create_publisher(String, '~/eki/command', 1)

    # Create TCP socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        node.get_logger().info(f"Successfully created TCP socket on ip:port {host}:{port}.")
    except socket.error as e:
        node.get_logger().fatal(f"Could not create socket. Error: {e}")
        sys.exit()

    node.get_logger().info(f"Waiting for connection.")
    # Bind the socket
    s.bind((host, port))
    s.listen(1)

    # Accept incoming connection
    conn, addr = s.accept()
    node.get_logger().info(f"TCP connection established with {addr}.")

    try:
        while rclpy.ok():
            timeout_count = 0
            while rclpy.ok() and timeout_count < max_timeout:
                time.sleep(0.001)  # FIXME: make this a ros2 node
                try:
                    # Create and send robot state as XML
                    str_data = create_eki_xml_rob(act_joint_pos)
                    msg = String()
                    msg.data = str(str_data)
                    eki_act_pub.publish(msg)
                    conn.send(str_data)  # Send data over TCP

                    # Receive the command message
                    recv_msg = conn.recv(1024)
                    if not recv_msg:
                        break  # No data received, close connection
                    node.get_logger().info(f"Received XML:\n{recv_msg.decode('utf-8')}")
                    msg = String()
                    msg.data = str(recv_msg)
                    eki_cmd_pub.publish(msg)

                    # Parse the received XML and update the joint position
                    des_joint_absolute = parse_eki_xml_sen(recv_msg)
                    if des_joint_absolute is not None:
                        act_joint_pos = des_joint_absolute  # Update the joint position
                    else:
                        continue
                    time.sleep(cycle_time / 2)

                except socket.timeout:
                    node.get_logger().info(f"Socket timed out.")
                    timeout_count += 1
                except socket.error as e:
                    node.get_logger().error(f"Socket error: {e}")
                    continue

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        # Clean up and close the connection
        node.get_logger().info(f"Shutting down '{node_name}' node.")
        conn.close()  # Close the TCP connection
        s.close()  # Close the socket

    rclpy.shutdown()


if __name__ == '__main__':
    main()
