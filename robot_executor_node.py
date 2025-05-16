#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import socket
import time
import minimalmodbus
import serial

# === ROBOT CONFIG ===
ROBOT_IP = '192.168.10.190'
ROBOT_PORT = 30001

# === GRIPPER SETUP ===
def setup_gripper():
    gripper = minimalmodbus.Instrument('/dev/ttyUSB0', 9)
    gripper.serial.baudrate = 115200
    gripper.serial.bytesize = 8
    gripper.serial.parity = serial.PARITY_NONE
    gripper.serial.stopbits = 1
    gripper.serial.timeout = 1
    gripper.write_register(0x03E8, 0x0001, 0)  # Activate
    return gripper

gripper = setup_gripper()

def send_script(script):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ROBOT_IP, ROBOT_PORT))
        sock.send((script + "\n").encode())
        sock.close()
        rospy.loginfo("✅ Script sent to robot.")
    except Exception as e:
        rospy.logerr(f"❌ Error sending script: {e}")

def close_gripper():
    gripper.write_register(0x03E8, 0x0900, 0)
    gripper.write_register(0x03E9, 0x00FF, 0)
    gripper.write_register(0x03EA, 0xFFFF, 0)
    time.sleep(2)

def open_gripper():
    gripper.write_register(0x03E8, 0x0900, 0)
    gripper.write_register(0x03E9, 0x0000, 0)
    gripper.write_register(0x03EA, 0xFFFF, 0)
    time.sleep(2)

def pick():
    rospy.loginfo("📦 Performing PICK")
    script = """def test_move():
    global P_start_p=p[0.3206, -0.15, 0.2809, 2.1919, -2.1463, -0.0555]
    global P_second_p=p[0.3206, -0.15, 0.2209, 2.1919, -2.1463, -0.0555]
    global P_third_p=p[0.3206, -0.15, 0.0525, 2.1919, -2.1463, -0.0555]
    movel(P_start_p, a=0.7, v=0.125)
    stopj(1)
    movel(P_second_p, a=0.7, v=0.125)
    stopj(1)
    movel(P_third_p, a=0.7, v=0.125)
    stopj(1)
end

test_move()"""
    send_script(script)
    time.sleep(5)
    close_gripper()

def place():
    rospy.loginfo("📤 Performing PLACE")
    script = """def place_move():
    global P1=p[0.3206, -0.15, 0.2209, 2.1919, -2.1463, -0.0555]
    global P2=p[0.3206, -0.15, 0.2809, 2.1919, -2.1463, -0.0555]
    global P3=p[0.3206, 0.2, 0.2809, 2.1919, -2.1463, -0.0555]
    movel(P1, a=0.7, v=0.125)
    stopj(1)
    movel(P2, a=0.7, v=0.125)
    stopj(1)
    movel(P3, a=0.7, v=0.125)
    stopj(1)
end

place_move()"""
    send_script(script)
    time.sleep(5)
    open_gripper()

def reset_position():
    rospy.loginfo("🏠 Returning to HOME position")
    script = """def go_home():
    global P_start_p=p[0.3206, -0.15, 0.2809, 2.1919, -2.1463, -0.0555]
    movel(P_start_p, a=0.7, v=0.125)
    stopj(1)
end

go_home()"""
    send_script(script)
    time.sleep(3)

# === ROS CALLBACK ===
def callback(msg):
    command = msg.data.lower()

    if "pick" in command:
        pick()
    elif "place" or "please" in command:
        place()
    elif "reset" in command:
        reset_position()
    else:
        rospy.logwarn(f"⚠️ Unknown command: {command}")

# === MAIN ROS NODE ===
def robot_executor_node():
    rospy.init_node('robot_executor_node')
    rospy.Subscriber("/voice_command", String, callback)
    rospy.loginfo("🤖 Robot executor node ready. Listening to /voice_command")
    rospy.spin()

if __name__ == '__main__':
    robot_executor_node()
