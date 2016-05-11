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
from barc.msg import ECU, Encoder
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from numpy import pi, cos, sin, eye, array, zeros
from observers import ekf
from system_models import f_DynBkMdl_3s, h_3s
from filtering import filteredSignal
from tf import transformations
from numpy import unwrap

# input variables
d_f 	    = 0         # steering angle
u_motor     = 0         # motor input

# raw measurement variables
yaw_prev    = 0
(roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z) = zeros(9)

# from encoder
v_x_enc 	= 0
t0 	        = time()
n_FL	    = 0                     # counts in the front left tire
n_FR 	    = 0                     # counts in the front right tire
n_FL_prev 	= 0
n_FR_prev 	= 0
r_tire 		= 0.04                  # radius from tire center to perimeter along magnets [m]
dx_qrt 	    = 2.0*pi*r_tire/4.0     # distance along quarter tire edge

vx_optic = 0
vy_optic = 0

# ecu command update
def ecu_callback(data):
    global u_motor, d_f
    u_motor	        = data.throttle
    d_f             = data.str_ang

# imu measurement update
def imu_callback(data):
    # units: [rad] and [rad/s]
    global roll, pitch, yaw, a_x, a_y, a_z, w_x, w_y, w_z
    global yaw_prev
    
    # get orientation from quaternion data, and convert to roll, pitch, yaw
    # extract angular velocity and linear acceleration data
    ori  = data.orientation
    quaternion  = (ori.x, ori.y, ori.z, ori.w)
    (roll, pitch, yaw) = transformations.euler_from_quaternion(quaternion)
    yaw         = unwrap(array([yaw_prev, yaw]), discont = pi)[1]
    yaw_prev    = yaw
    
    # extract angular velocity and linear acceleration data
    w_x = data.angular_velocity.x
    w_y = data.angular_velocity.y
    w_z = data.angular_velocity.z
    a_x = data.linear_acceleration.x
    a_y = data.linear_acceleration.y
    a_z = data.linear_acceleration.z

# optic flow update
def optic_flow_callback(data):
    global vx_optic,vy_optic
    vx_optic = data.x
    vy_optic = data.y

# encoder measurement update
def enc_callback(data):
	global v_x_enc, d_f, t0
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

		# update encoder v_x, v_y measurements
		# only valid for small slip angles, still valid for drift?
		v_x_enc 	= (v_FL + v_FR)/2.0*cos(d_f)

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
    Subscriber('vel_est', Vector3, optic_flow_callback)
    state_pub 	= Publisher('state_estimate', Vector3, queue_size = 10)

	# get vehicle dimension parameters
    L_a     = get_param("state_estimation/L_a")       # distance from CoG to front axel
    L_b     = get_param("state_estimation/L_b")       # distance from CoG to rear axel
    m       = get_param("state_estimation/m")         # mass of vehicle
    I_z     = get_param("state_estimation/I_z")       # moment of inertia about z-axis
    vhMdl   = (L_a, L_b, m, I_z)

    # get encoder parameters
    dt_vx   = get_param("state_estimation/dt_vx")     # time interval to compute v_x

    # get tire model
    B       = get_param("state_estimation/B")
    C       = get_param("state_estimation/C")
    mu      = get_param("state_estimation/mu")
    TrMdl   = ([B,C,mu],[B,C,mu])

    # get vehicle model parameters
    a0      = get_param("state_estimation/air_drag_coeff")
    b0      = get_param("state_estimation/input_gain")
    Ff      = get_param("state_estimation/friction")

    # get EKF observer properties
    q_std   = get_param("state_estimation/q_std")             # std of process noise
    r_std   = get_param("state_estimation/r_std")             # std of measurementnoise
    v_x_min = get_param("state_estimation/v_x_min")  # minimum velociy before using EKF

	# set node rate
    loop_rate 	= 50
    dt 		    = 1.0 / loop_rate
    rate 		= Rate(loop_rate)
    t0 		    = time()

    # estimation variables for Luemberger observer
    z_EKF       = array([1.0, 0.0, 0.0])

    # estimation variables for EKF
    P           = eye(3)                # initial dynamics coveriance matrix
    Q           = (q_std**2)*eye(3)     # process noise coveriance matrix
    R           = (r_std**2)*eye(2)     # measurement noise coveriance matrix

    # filtered signal for longitudinal velocity
    p_filter    = get_param("state_estimation/p_filter")
    v_x_filt    = filteredSignal(a = p_filter, method='lp')   # low pass filter

    while not is_shutdown():

		# publish state estimate
        (v_x, v_y, r) = z_EKF           # note, r = EKF estimate yaw rate

        # publish information
        state_pub.publish( Vector3(v_x, v_y, r) )

        # update filtered signal
        v_x_filt.update(v_x_enc)
        v_x_est = v_x_filt.getFilteredSignal() 

        # apply EKF
        if v_x_est > v_x_min:
            # get measurement
            y       = array([v_x_est, w_z])
            FxR     = b0*u_motor

            # define input
            u       = array([ d_f, FxR ])

            # build extra arguments for non-linear function
            F_ext = array([ a0, Ff ]) 
            args = (u, vhMdl, TrMdl, F_ext, dt) 

            # apply EKF and get each state estimate
            (z_EKF,P) = ekf(f_DynBkMdl_3s, z_EKF, P, h_3s, y, Q, R, args )
        else:
            z_EKF[0] = float(v_x_enc)
            z_EKF[2] = float(w_z)
        
		# wait
        rate.sleep()

if __name__ == '__main__':
	try:
	   state_estimation()
	except ROSInterruptException:
		pass
