#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------

import rospy
from barc.msg import ECU
from data_service.msg import TimeData
from math import pi,sin
import time
import serial
from numpy import zeros, hstack, cos, array, dot, arctan, sign
from numpy import unwrap
from input_map import angle_2_servo, servo_2_angle
from pid import PID

# pid control for constrant yaw angle 
yaw0        = 0      
read_yaw0   = False
yaw_prev    = 0      
yaw         = 0
err         = 0
def imu_callback(data):
    global yaw0, read_yaw0, yaw_prev, yaw, err

    # extract yaw angle
    (_,_,yaw, _,_,_, _,_,_) = data.value

    # save initial measurements
    if not read_yaw0:
        read_yaw0 = True
        yaw0    = yaw
    else:
        temp        = unwrap(array([yaw_prev, yaw]))
        yaw         = temp[1]
        yaw_prev    = yaw

    err = yaw - yaw0


#############################################################
def openloopMotor(t_i, t_f):
    t_start     = 1

    # rest
    if t_i < t_start:
        u_motor     = opt.neutral

    # start moving
    elif (t_i >= t_0) and (t_i < t_f):
        u_motor     = opt.speed

    # set straight and stop
    else:
        u_motor     = opt.neutral

    return (u_motor, str_ang)

#############################################################
def main_auto():
    # initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    nh = rospy.Publisher('ecu', ECU, queue_size = 10)
    rospy.Subscriber('imu', TimeData, imu_callback)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    dt      = 1.0 / rateHz
    t_i     = 0.0

    # use simple pid control to keep steering straight
    p 		= rospy.get_param("controller/p")
    i 		= rospy.get_param("controller/i")
    d 		= rospy.get_param("controller/d")
    t_f     = rospy.get_param("controller/t_f")
    pid     = PID(P=p, I=i, D=d)
    pid.setPoint(0)

    # main loop
    while not rospy.is_shutdown():
        # get steering wheel command
        d_f             = pid.update(yaw, dt)
        u_motor         = openloopMotor(t_i, t_f)
			
        # send command signal
        ecu         = ECU(u_motor, d_f)
        nh.publish(ecu_cmd)
	
        # wait
        t_i += dt
        rate.sleep()

#############################################################
if __name__ == '__main__':
	try:
		main_auto()
	except rospy.ROSInterruptException:
		pass
