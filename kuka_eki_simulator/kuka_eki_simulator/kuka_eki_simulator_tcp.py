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

def create_eki_xml_rob(act_joint_pos, command_id="1"):
    q = act_joint_pos
    qd = [0.0] * 6  # Joint velocities
    eff = [0.0] * 6  # Joint torques

    root = ET.Element('RobotState')

    # Command
    command = ET.SubElement(root, 'Command')
    command.set('Id', str(command_id))

    # Joint positions
    position = ET.SubElement(root, 'Position')
    joint = ET.SubElement(position, 'Joint')
    for i in range(6):
        joint.set(f'A{i+1}', str(q[i]))
    #joint.set('A7', "0.0")  # Placeholder for A7

    # Cartesian positions (placeholder values)
    cartesian = ET.SubElement(position, 'Cartesian')
    for axis in ['X', 'Y', 'Z', 'A', 'B', 'C']:
        cartesian.set(axis, "0.0")

    # Joint velocities
    velocity = ET.SubElement(root, 'Velocity')
    for i in range(6):
        velocity.set(f'A{i+1}', str(qd[i] / max_vel))

    # Joint torques
    torque = ET.SubElement(root, 'Torque')
    for i in range(6):
        torque.set(f'A{i+1}', str(eff[i]))

    # Gripper state (placeholders)
    gripper = ET.SubElement(root, 'Gripper')
    jaw = ET.SubElement(gripper, 'Jaw')
    jaw.set('Position', "0.0")
    jaw.set('Status', "0")
    vacuum = ET.SubElement(gripper, 'Vacuum')
    vacuum.set('Suction', "0")
    vacuum.set('Force1', "0.0")
    vacuum.set('Force2', "0.0")
    vacuum.set('Cylinder', "0")

    # Info (placeholder)
    info = ET.SubElement(root, 'Info')
    info.set('Code', "0")
    info.set('Message', "OK")

    # Error (placeholder)
    error = ET.SubElement(root, 'Error')
    error.set('Code', "0")
    error.set('Message', "")

    return ET.tostring(root, short_empty_elements=False)


def parse_eki_xml_sen(data):
    try:
        result = {}

        # Parse the XML data
        tree = ET.ElementTree(ET.fromstring(data))
        root = tree.getroot()

        # Extract command ID (from <RobotCommand Id="...">)
        command_id = root.attrib.get('Id')
        if command_id is None:
            raise ValueError("Missing 'Id' attribute in <RobotCommand> element")
        result['command_id'] = int(command_id)

        # Extract Mode from <Move> element
        move_element = root.find('.//Move')
        if move_element is None:
            raise ValueError("Missing <Move> element")
        mode = int(move_element.attrib.get('Mode', -1))  # Default to -1 if Mode is missing

        # Extract joint values (from <Move><Joint A1="0.000000" A2="0.000000" ...>)
        joint = move_element.find('Joint')
        if joint is None:
            raise ValueError("Missing <Joint> element in <Move> section")

        joint_values = []
        if mode == 1: # Joint mode --> extract joint values from the XML
            for axis in ['A1', 'A2', 'A3', 'A4', 'A5', 'A6']:
                axis_value = joint.attrib.get(axis)
                if axis_value is None:
                    raise ValueError(f"Missing joint value for {axis}")
                joint_values.append(float(axis_value))
        else: # Cartesian mode --> fill joint values with 0.0, since no IK is implemented
            print(f"[Warning] Cartesian mode detected, using 0.0 joint values, because no IK is implemented.")
            joint_values = [0.0] * 6

        result['joint_positions'] = np.array(joint_values, dtype=np.float64)

        return result

    except Exception as e:
        print(f"[Error] Failed to parse RobotCommand: {e}")
        return None



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
    act_command_id = -1
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
                    str_data = create_eki_xml_rob(act_joint_pos, act_command_id+1)
                    msg = String()
                    msg.data = str(str_data)
                    eki_act_pub.publish(msg)
                    conn.send(str_data)  # Send data over TCP
                    node.get_logger().info(f"Sent XML:\n{str_data.decode('utf-8')}")

                    # Receive the command message
                    recv_msg = conn.recv(1024)
                    if not recv_msg:
                        break  # No data received, close connection
                    node.get_logger().info(f"Received XML:\n{recv_msg.decode('utf-8')}")
                    msg = String()
                    msg.data = str(recv_msg)
                    eki_cmd_pub.publish(msg)

                    # Parse the received XML and update the joint position and command ID
                    parsed_data = parse_eki_xml_sen(recv_msg)

                    if parsed_data is not None and parsed_data['joint_positions'] is not None:
                        act_joint_pos = parsed_data['joint_positions']
                        act_command_id = parsed_data['command_id']
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
