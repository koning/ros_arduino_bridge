#!/usr/bin/env python

"""
    IMU class for the arudino_python package
    
    Created for the Programming Robots Study Group http://robotgarden.org/prsg
    Copyright (c) 2016 Joe Koning.  All rights reserved.


    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
from arduino_sensors import Sensor
from sensor_msgs.msg import Imu as msgImu

    
class Imu(Sensor):
    def __init__(self, controller, name, addr, rate, frame_id, **kwargs):
        self.controller = controller
        self.name = name
        self.addr = addr 
        self.rate = rate

        self.frame_id = frame_id

        self.cal_offset = 0.0
        self.orientation = 0.0
        self.cal_buffer =[]
        self.cal_buffer_length = 1000
        self.imu_data = msgImu(header=rospy.Header(frame_id=frame_id))
        self.imu_data.orientation_covariance = [1,0,0,0,1,0,0,0,1]
        self.imu_data.angular_velocity_covariance = [1,0,0,0,1,0,0,0,1]
        self.imu_data.linear_acceleration_covariance = [1,0,0,0,1,0,0,0,1]
        #self.accel_measurement_range = rospy.get_param('~accel_measurement_range', 2.0) 
        #self.gyro_measurement_range = rospy.get_param('~gyro_measurement_range', 150.0) 
        #self.gyro_scale_correction = rospy.get_param('~gyro_scale_correction', 1.35)
        self.imu_pub = rospy.Publisher('~sensor/' + self.name , msgImu, queue_size=1)
        self.imu_pub_raw = rospy.Publisher('~sensor/' +self.name + '_raw', msgImu, queue_size=1)
        
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            try:
                 self.values = self.read_state()
            except:
                 return
    
            # Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            
            self.t_next = now + self.t_delta
    
    def acc_to_angles(self, ax, ay, az):
        " Convert accelerations into angles in radian/sec"
        az2 = az**2
        #pitch = atan((ax/(ay**2) + az**2)  
        #roll = atan((ay/(ax**2) + az**2)  

class GY85(Imu):
    def __init__(self, *args, **kwargs):
        super(GY85, self).__init__(*args, **kwargs)
                
        self.controller.init_imu("GY85", self.addr)
    def read_state(self):
        '''This Imu returns linear accelerations, gyro and magnetometer data'''
        values = self.controller.get_imu_values(self.addr, vtype=int)
        gconv = 9.8 # [m/s]/[g]
        self.imu_data.linear_acceleration_x = gconv*self.values[0]
        self.imu_data.linear_acceleration_y = gconv*self.values[1]
        self.imu_data.linear_acceleration_z = gconv*self.values[2]
            
class MPU6050(Imu):
    def __init__(self, *args, **kwargs):
        super(MPU6050, self).__init__(*args, **kwargs)
        #Full scale is [-32678,32678]
        #The default settings are +/-2g and 250 rad/sec.
        self.aconv = 2*9.8/32768 # [m/s^2]
        self.gconv = 250./32768  # [rad/s]
                
        self.controller.init_imu("MPU6050", self.addr)
    def read_state(self):
        '''This Imu returns linear accelerations and gyro information
        '''
        values = self.controller.get_imu_values(self.addr, vtype=int)
        self.imu_data.linear_acceleration_x = aconv*self.values[0]
        self.imu_data.linear_acceleration_y = aconv*self.values[1]
        self.imu_data.linear_acceleration_z = aconv*self.values[2]
        self.imu_data.angular_acceleration_x = gconv*self.values[3]
        self.imu_data.angular_acceleration_y = gconv*self.values[4]
        self.imu_data.angular_acceleration_z = gconv*self.values[5]

if __name__ == '__main__':
    myController = Controller()
    mySensor = MPU6050(myController, "MPU6050", rate=10)

