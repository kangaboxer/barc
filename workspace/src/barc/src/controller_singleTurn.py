#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu)  Development of the web-server app Dator was
# based on an open source project by Bruce Wootton, with contributions from
# Kiet Lam (kiet.lam@berkeley.edu)
# ---------------------------------------------------------------------------

import rospy
from barc.msg import ECU

from math import pi,sin
from numpy import array, dot, cos, tan, arctan, ceil 
from numpy import array, float32, zeros, arcsin, unwrap
from input_map import angle_2_servo, servo_2_angle
from pid import PID
import numpy as np

# global constants
g = 9.81

# pid control for constrant yaw angle 
yaw0        = 0      
read_yaw0   = False
yaw_prev    = 0      
yaw         = 0
err         = 0

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global yaw0, read_yaw0, yaw_prev, yaw, err
    global yaw_prev
    
    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori  = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw

    # save initial measurements
    if not read_yaw0:
        read_yaw0 = True
        yaw0    = yaw
    else:
        temp        = unwrap(array([yaw_prev, yaw]))
        yaw         = temp[1]
        yaw_prev    = yaw

#############################################################
def SingleTurn(rate, t_i, pid, time_params, input_params):
    # unpack parameters
    (t_0, t_turn, dt_motorMax, t_f, dt)             = time_params
    (motorPWM_target, motorPWM_max, str_ang_target) = input_params 

    # rest
    if t_i < t_0:
        servoCMD    = 90
        motorCMD    = 90

    # move forward at increasing speeds
    elif (t_i < t_turn):
        rospy.loginfo(yaw)
        servoCMD    = angle_2_servo( pid.update(yaw, dt) )
        step_up     = 95  + np.round( float(t_i - t_0) )
        motorCMD    = np.min([step_up, motorPWM_target])

    # apply turn
    elif (t_i < t_turn + dt_motorMax):
        servoCMD    = angle_2_servo(str_ang_target)
        motorCMD    = motorPWM_target

    # apply max motor
    elif (t_i < t_f):
        servoCMD    = angle_2_servo(str_ang_target)
        motorCMD    = motorPWM_max
        
    # stop experiment
    else:
        servoCMD    = 90
        motorCMD    = 90

    return (motorCMD, servoCMD)

#############################################################
def main_auto():
	# initialize ROS node
    rospy.init_node('auto_mode', anonymous=True)
    rospy.Subscriber('imu', TimeData, imu_callback)
    nh      = rospy.Publisher('ecu', ECU, queue_size = 10)

	# set node rate
    rateHz  = 50
    rate 	= rospy.Rate(rateHz)
    dt      = 1.0 / rateHz
    t_i     = 0

    # use simple pid control to keep steering straight initially
    p 		= rospy.get_param("controller/p")
    i 		= rospy.get_param("controller/i")
    d 		= rospy.get_param("controller/d")
    pid     = PID(P=p, I=i, D=d)

    t_0                 = rospy.get_param("controller/t_0")
    t_turn              = rospy.get_param("controller/t_turn")
    dt_motorMax         = rospy.get_param("controller/dt_motorMax")
    t_f                 = rospy.get_param("controller/t_f")
    str_ang_target 	    = rospy.get_param("controller/steering_angle")
    motorPWM_target     = rospy.get_param("controller/motorPWM_target")
    motorPWM_max        = rospy.get_param("controller/motorPWM_max")
    time_params         = (t_0, t_turn, dt_motorMax, t_f, dt)
    input_params        = (motorPWM_target, motorPWM_max, str_ang_target)

    t_i                 = 0.0
    setReference    = False

    # main loop
    while not rospy.is_shutdown():

        # OPEN LOOP 
        if read_yaw0:
            # set reference angle
            if not setReference:
                pid.setPoint(yaw0)
                setReference = True
            # apply open loop command
            else:
                (motorCMD, servoCMD)    = SingleTurn(rate, t_i, pid, time_params, input_params)
                ecu_cmd = ECU(motorCMD, servoCMD)
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
