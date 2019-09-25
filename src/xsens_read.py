#!/usr/bin/env python
import serial
import struct
import rospy
from sensor_msgs.msg import Imu ,MagneticField, Temperature
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import numpy as np

rospy.init_node("xsens")
uart = serial.Serial("/dev/ttyAMA0",115200)
ImuPub = rospy.Publisher("/xsens/imu/data",Imu,queue_size=1)
magPub = rospy.Publisher("/xsens/mag",MagneticField,queue_size=1)
xsensVelPub = rospy.Publisher("/xsens/vel",Vector3,queue_size=1)
tempPub = rospy.Publisher("/xsens/temp",Temperature,queue_size=1)
try:
    while(uart.isOpen()):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "/xsens/imu"
        imuData = Imu()
        xsensVel = Vector3()
        magData = MagneticField()
        temperature = Temperature()
        for i in range(2):
            byte = hex(int(uart.read(1).encode("hex"),16))  #otsekaem do baita
            if (byte == "0xfa"):
                byte = hex(int(uart.read(1).encode("hex"),16))
                if (byte == "0xff"):
                    first_byte = hex(int(uart.read(1).encode("hex"),16))
                    second_byte = hex(int(uart.read(1).encode("hex"),16))
                    package_length = (int(second_byte,16))
                    orientation_hex = (hex(int(uart.read(package_length).encode("hex"),16)))
                    uart.flushInput()
                    if package_length == 15:
                        roll=struct.unpack('!f', orientation_hex[8:16].decode("hex"))[0]
                        pitch = struct.unpack('!f', orientation_hex[16:24].decode("hex"))[0]
                        yaw = struct.unpack('!f', orientation_hex[24:32].decode("hex"))[0]
                        
                        roll = roll*np.pi/180
                        pitch = pitch*np.pi/180
                        yaw = yaw*np.pi/180
                        quanternion = quaternion_from_euler(roll,pitch,yaw)
                        imuData.orientation.x = quanternion[0]
                        imuData.orientation.y = quanternion[1]
                        imuData.orientation.z = quanternion[2]
                        imuData.orientation.w = quanternion[3]
                    if package_length == 67:
                        imuData.header = header
                        imuData.linear_acceleration.x=struct.unpack('!f', orientation_hex[8:16].decode("hex"))[0]
                        imuData.linear_acceleration.y = struct.unpack('!f', orientation_hex[16:24].decode("hex"))[0]
                        imuData.linear_acceleration.z = struct.unpack('!f', orientation_hex[24:32].decode("hex"))[0]

                        xsensVel.x = struct.unpack('!f', orientation_hex[38:46].decode("hex"))[0]
                        xsensVel.y = struct.unpack('!f', orientation_hex[46:54].decode("hex"))[0]
                        xsensVel.z = struct.unpack('!f', orientation_hex[54:62].decode("hex"))[0]

                        imuData.angular_velocity.x=struct.unpack('!f', orientation_hex[68:76].decode("hex"))[0]
                        imuData.angular_velocity.y = struct.unpack('!f', orientation_hex[76:84].decode("hex"))[0]
                        imuData.angular_velocity.z = struct.unpack('!f', orientation_hex[84:92].decode("hex"))[0]

                        magData.header = header
                        magData.magnetic_field.x=struct.unpack('!f', orientation_hex[98:106].decode("hex"))[0]
                        magData.magnetic_field.y = struct.unpack('!f', orientation_hex[106:114].decode("hex"))[0]
                        magData.magnetic_field.z = struct.unpack('!f', orientation_hex[114:122].decode("hex"))[0]

                        temperature.header = header
                        temperature.temperature= struct.unpack('!f', orientation_hex[128:136].decode("hex"))[0]
                        #print(temperature.temperature)
        ImuPub.publish(imuData)
        xsensVelPub.publish(xsensVel)
        tempPub.publish(temperature)
        magPub.publish(magData)        
                
except KeyboardInterrupt:
    uart.close()


     
         


    
        

    


   
    
























    
