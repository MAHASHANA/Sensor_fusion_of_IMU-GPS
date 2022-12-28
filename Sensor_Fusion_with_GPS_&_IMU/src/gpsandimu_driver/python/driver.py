#!/usr/bin/env python
#-*- coding: utf-8 -*-
from imu_driver.msg import imu_msg
import serial
import rospy
import utm
from std_msgs.msg import String
import numpy as np 
import sys
import time
rospy.init_node('mynode')
port = rospy.get_param("~port_number")

ser = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
pub = rospy.Publisher('gps_imu', imu_msg, queue_size=10)

t = rospy.Time.now()
#t2= t.split(".")
msg = imu_msg()


while True: 
    imc = str(ser.readline())
    sac = str(ser.readline())
    
    if 'YMR' in str(imc):
        ser.write("VNWRG,06,0*XX".encode('utf-8'))
        ser.write("VNWRG,07,40*XX".encode('utf-8'))
        pl = imc.split(",")
        print(imc)
        yaw = float(pl[1])*(np.pi/180)
        pitch = float(pl[2])*(np.pi/180)
        roll = float(pl[3])*(np.pi/180)
        mag_x = float(pl[4])
        mag_y = float(pl[5])
        mag_z = float(pl[6])
        accl_x = float(pl[7])
        accl_y = float(pl[8])
        accl_z = float(pl[9])
        angl_x = float(pl[10])
        angl_y = float(pl[11])
        angl_z1 =pl[12]
        angl_z = float(angl_z1[:-8])
        
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        msg.IMU.orientation.x = qx
        msg.IMU.orientation.y = qy
        msg.IMU.orientation.z = qz
        msg.IMU.orientation.w = qw
        msg.IMU.angular_velocity.x = angl_x
        msg.IMU.angular_velocity.y = angl_y
        msg.IMU.angular_velocity.z = angl_z
        msg.IMU.linear_acceleration.x= accl_x
        msg.IMU.linear_acceleration.y= accl_y
        msg.IMU.linear_acceleration.z= accl_z
        msg.MagField.magnetic_field.x = mag_x
        msg.MagField.magnetic_field.y = mag_y
        msg.MagField.magnetic_field.z = mag_z
        msg.VNYMR = imc
        imc = str(ser.readline())
    elif 'GGA' in str(sac):
        gl = sac.split(",")
        print(sac)
        H = float(gl[1])
        x = float(gl[2])
        a = float(gl[4])
        
        lat = str(x)
        lon = str(a)
        time = str(H)
#print (float(lat[:2])+float(lat[2:])/60)
        sec = float(time[:2])*60*60+float(time[2:4])*60+float(time[4:6])
        nsec = (float(time[6:]))*10e6
        y = float(lat[:2])+float(lat[2:])/60
        z = (float(lon[:3])+float(lon[3:])/60)*-1
        gap = utm.from_latlon(float(y), float(z))
        msg.Latitude = y
        msg.Longitude = z
        msg.UTM_northing = gap[0]
        msg.UTM_easting = gap[1]
        msg.Altitude = float(gl[8])
        msg.Zone = gap[2]
        msg.Letter = gap[3]
        msg.Header.stamp.secs = int(sec)
        msg.Header.stamp.nsecs = int(nsec)
        msg.Header.frame_id = "GPSandimu_FRAME"
        msg.GPGGA = sac 
        print(sec, nsec)
        print(f"lat long zone {gap}")
        sac = str(ser.readline())
    
    pub.publish(msg)

            
            






