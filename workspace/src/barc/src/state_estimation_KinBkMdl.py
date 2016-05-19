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

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException
from time import time
from barc.msg import ECU, Encoder, Z_KinBkMdl
from sensor_msgs.msg import Imu
from numpy import pi, cos, sin, eye, array, zeros, unwrap
from observers import ekf
from system_models import f_KinBkMdl, h_KinBkMdl
from filtering import filteredSignal
from tf import transformations

# input variables [default values]
d_f 	    = 0         # steering angle [deg]
u_motor     = 0         # motor input

# raw measurement variables
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)
yaw_prev    = 0
yaw0        = 0      
read_yaw0   = False
yaw_local   = 0

# from encoder
v 	        = 0
t0 	        = time()
n_FL	    = 0                     # counts in the front left tire
n_FR 	    = 0                     # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt 	    = 2.0*pi*r_tire/4.0     # distance along quarter tire edge [m]

# ecu command update
def ecu_callback(data):
    global u_motor, d_f
    u_motor     = data.throttle
    d_f         = data.str_ang

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev, yaw0, read_yaw0, yaw_local

    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori         = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)

    # save initial measurements
    if not read_yaw0:
        read_yaw0   = True
        yaw_prev    = yaw
        yaw0        = yaw
    
    # unwrap measurement
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw
    yaw_local   = yaw - yaw0
    
    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z

# encoder measurement update
def enc_callback(data):
	global v, d_f, t0
	global n_FL, n_FR, n_FL_prev, n_FR_prev

	n_FL = data.FL
	n_FR = data.FR

	# compute time elapsed
	tf = time()
	dt = tf - t0
	
	# if enough time elapse has elapsed, estimate v_x
	dt_min = 0.20
	if dt >= dt_min:
		# compute speed :  speed = distance / time
		v_FL = float(n_FL- n_FL_prev)*dx_qrt/dt
		v_FR = float(n_FR- n_FR_prev)*dx_qrt/dt
		v 	 = (v_FL + v_FR)/2.0

		# update old data
		n_FL_prev   = n_FL
		n_FR_prev   = n_FR
		t0 	        = time()


# state estimation node
def state_estimation():
	# initialize node
    init_node('state_estimation', anonymous=True)

    # topic subscriptions / publications
    Subscriber('imu/data', Imu, imu_callback)
    Subscriber('encoder', Encoder, enc_callback)
    Subscriber('ecu', ECU, ecu_callback)
    state_pub 	= Publisher('state_estimate', Z_KinBkMdl, queue_size = 10)

	# get vehicle dimension parameters
    L_a         = get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b         = get_param("state_estimation/L_b")       # distance from CoG to rear axel
    b0          = get_param("state_estimation/input_gain")
    vhMdl       = (L_a, L_b)

    # get encoder parameters
    dt_vx       = get_param("state_estimation/dt_vx")     # time interval to compute v_x

    # get EKF observer properties
    q_std       = get_param("state_estimation/q_std")             # std of process noise
    r_std       = get_param("state_estimation/r_std")             # std of measurementnoise

	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= Rate(loop_rate)
    t0 			= time()

    # estimation variables for Luemberger observer
    z_EKF       = zeros(4) 

    # estimation variables for EKF
    P           = eye(4)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(4)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix

    while not is_shutdown():
		# publish state estimate
        (x, y, psi, vel) = z_EKF          

        # publish information
        state_pub.publish( Z_KinBkMdl(x, y, psi, vel) )

        # collect measurements, inputs, system properties
        y_meas  = array([ yaw_local, v])
        u       = array([ d_f, b0*u_motor ])
        args    = (u,vhMdl,dt) 

        # apply EKF and get each state estimate
        (z_EKF,P) = ekf(f_KinBkMdl, z_EKF, P, h_KinBkMdl, y_meas, Q, R, args )

		# wait
        rate.sleep()

if __name__ == '__main__':
	try:
	   state_estimation()
	except ROSInterruptException:
		pass
