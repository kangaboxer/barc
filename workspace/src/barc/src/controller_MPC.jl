#!/usr/bin/env julia

#=
 Licensing Information: You are free to use or extend these projects for 
 education or reserach purposes provided that (1) you retain this notice
 and (2) you provide clear attribution to UC Berkeley, including a link 
 to http://barc-project.com

 Attibution Information: The barc project ROS code-base was developed
 at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
 (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
 by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
 based on an open source project by Bruce Wootton
=# 

using RobotOS
@rosimport barc.msg: ECU, Encoder, Ultrasound, Z_KinBkMdl
@rosimport data_service.msg: TimeData
@rosimport geometry_msgs.msg: Vector3
rostypegen()
using barc.msg
using data_service.msg
using geometry_msgs.msg
using JuMP
using Ipopt

# define model parameters
L_a     = 0.125         # distance from CoG to front axel
L_b     = 0.125         # distance from CoG to rear axel
dt      = 0.1           # time step of system

# preview horizon
N       = 5

# define targets [generic values]
x_ref   = 1
y_ref   = 0

# define decision variables 
# states: position (x,y), yaw angle, and velocity
# inputs: acceleration, steering angle 
println("Creating kinematic bicycle model ....")
mdl     = Model(solver = IpoptSolver(print_level=3))
@variable( mdl, x[1:(N+1)] , start = 0.0)
@variable( mdl, y[1:(N+1)], start = 0.0)
@variable( mdl, psi[1:(N+1)], start = 0.0 )
@variable( mdl, v[1:(N+1)], start = 0.0 )
@variable( mdl, a[1:N] )
@variable( mdl, d_f[1:N] )

# define objective function
@NLobjective(mdl, Min, (x[N+1] - x_ref)^2 + (y[N+1] - y_ref)^2 )

# define constraints
# define system dynamics
# Reference: R.Rajamani, Vehicle Dynamics and Control, set. Mechanical Engineering Series,
#               Spring, 2011, page 26
# initial condition
@NLparameter(mdl, x0     == 0); @NLconstraint(mdl, x[1]     == x0);
@NLparameter(mdl, y0     == 0); @NLconstraint(mdl, y[1]     == y0);
@NLparameter(mdl, psi0   == 0); @NLconstraint(mdl, psi[1]   == psi0 );
@NLparameter(mdl, v0     == 0); @NLconstraint(mdl, v[1]     == v0);

# model dynamics
@NLexpression(mdl,  bta[i=1:N], atan( L_a / (L_a + L_b) * tan(d_f[i]) ) )
for i in 1:N
    @NLconstraint(mdl, x[i+1]    == x[i]      + dt*(v[i]*cos( psi[i] + bta[i] ))  )
    @NLconstraint(mdl, y[i+1]    == y[i]      + dt*(v[i]*sin( psi[i] + bta[i] ))  )
    @NLconstraint(mdl, psi[i+1]  == psi[i]    + dt*(v[i]/L_b * sin(bta[i]))  )
    @NLconstraint(mdl, v[i+1]    == v[i]      + dt*(a[i])  )
    @NLconstraint(mdl, a[i]  <= 5 )
end

# status update
println("initial solve ...")
solve(mdl)
println("finished initial solve!")

function SE_callback(msg::Z_KinBkMdl)
    # update mpc initial condition 
    setvalue(x0,    msg.x)
    setvalue(y0,    msg.y)
    setvalue(psi0,  msg.psi)
    setvalue(v0,    msg.v)
end

function main()
    # initiate node, set up publisher / subscriber topics
    init_node("mpc")
    pub = Publisher("ecu", ECU, queue_size=10)
    s1  = Subscriber("state_estimate", Z_KinBkMdl, SE_callback, queue_size=10)
    
    # get model parameters
    b0      = get_param("state_estimation/input_gain")

    # set rate
    loop_rate = Rate(10)

    while ! is_shutdown()
        # run mpc, publish command
        solve(mdl)

        # get optimal solutions
        a_opt   = getvalue(a[1])
        d_f_opt = getvalue(d_f[1])

        # apply ecu command
        u_motor = a_opt/b0            
        cmd     = ECU(u_motor, d_f_opt)
        publish(pub, cmd)

        # sleep
        rossleep(loop_rate)
    end
end

if ! isinteractive()
    main()
end
