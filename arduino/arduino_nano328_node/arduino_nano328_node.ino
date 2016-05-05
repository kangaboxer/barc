/* ---------------------------------------------------------------------------
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
# --------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
WARNING:
* Be sure to have all ultrasound sensors plugged in, otherwise the pins may get stuck in
  some float voltage
# --------------------------------------------------------------------------- */


// include libraries
#include <ros.h>
#include <barc/Ultrasound.h>
#include <barc/Encoder.h>
#include <barc/ECU.h>
#include <Servo.h>
#include "Maxbotix.h"

// Number of encoder counts on tires
// F = front, B = back, L = left, R = right
volatile int FL_count = 0;
volatile int FR_count = 0;
volatile int BL_count = 0;
volatile int BR_count = 0;

// Set up ultrasound sensors
// fr = front, bk = back, rt = right, lt = left
/*
Maxbotix us_fr(14, Maxbotix::PW, Maxbotix::LV); 
Maxbotix us_bk(15, Maxbotix::PW, Maxbotix::LV);
Maxbotix us_rt(16, Maxbotix::PW, Maxbotix::LV);
Maxbotix us_lt(17, Maxbotix::PW, Maxbotix::LV);
*/

//encoder pins: pins 2,3 are hardware interrupts
const int encPinA = 2;
const int encPinB = 3;

// Actuator pins: 5,6
// <Servo> data type performs PWM
// Declare variables to hold actuator commands
Servo motor;
Servo steering;
const int motorPin = 10;
const int servoPin = 11;
int motorCMD;
int servoCMD;

// max / min steering angle and motor commands
int str_ang_max = 30;
int str_ang_min = -30;
int motor_max = 15;
int motor_min = -15;

// variable for time
volatile unsigned long dt;
volatile unsigned long t0;

// ROS node and message variable initialization
ros::NodeHandle nh;                                         // create ros handle
barc::Ultrasound ultrasound;                                // declare ROS message variables 
barc::ECU ecu;
barc::Encoder encoder;
ros::Publisher pub_encoder("encoder", &encoder);            // set up publishers
ros::Publisher pub_ultrasound("ultrasound", &ultrasound);

/**************************************************************************
function    : ecu_callback
purpose     : process the ecu commands from the odroid 
parameters  :
    * ecu message variable
return 
    * ecu message variable
**************************************************************************/
/void ecu_callback(const barc::ECU& ecu){
    // deconstruct esc message
    motorCMD    = ecu.motor_pwm;
    servoCMD    = ecu.servo_pwm;
    
    // saturate commands
    motorCMD    = saturate(motorCMD, motor_min, motor_max);
    servoCMD    = saturate(servoCMD, str_ang_min, str_ang_max);

    // apply commands to motor and servo
    motor.write( motorCMD );
    steering.write(  servoCMD );
}
ros::Subscriber<barc::ECU>sub_ecu("ecu", ecu_callback);

/**************************************************************************
ARDUINO INITIALIZATION
**************************************************************************/
void setup()
{
    // Set up encoder sensors
    pinMode(encPinA, INPUT_PULLUP);
    pinMode(encPinB, INPUT_PULLUP);
    attachInterrupt(0, FL_inc, CHANGE); // args = (digitalPintoInterrupt, ISR, mode), mode set = {LOW, CHANGE, RISING, FALLING}
                                        // pin 0 = INT0, which is pin D2
    attachInterrupt(1, FR_inc, CHANGE);     //pin 1 = INT1, which is pin D3

     // Set up actuators
    motor.attach(motorPin);
    steering.attach(servoPin);

    // Start ROS node
    nh.initNode();

    // Publish / Subscribe to topics
    nh.advertise(pub_ultrasound);
    nh.advertise(pub_encoder);
    nh.subscribe(sub_ecu);

    // Arming ESC, 1 sec delay for arming and ROS
    motor.write(theta_center);
    steering.write(theta_center);
    delay(1000);
    t0 = millis();
}


/**************************************************************************
ARDUINO MAIN lOOP
**************************************************************************/
void loop()
{
    // compute time elapsed (in ms)
    dt = millis() - t0;

    // publish measurements
    if (dt > 50) {
        // publish encodeer measurement
        encoder.FL = FL_count;
        encoder.FR = FR_count;
        encoder.BL = 0;
        encoder.BR = 0;
        pub_encoder.publish(&encoder);

        // publish ultra-sound measurement
        /*
        ultrasound.front = us_fr.getRange();
        ultrasound.back = us_bk.getRange();
        ultrasound.right = us_rt.getRange();
        ultrasound.left = us_lt.getRange();
        */
    
        pub_ultrasound.publish(&ultrasound);
        t0 = millis();
    }

    nh.spinOnce();
}

/**************************************************************************
function    : {FL,FR,BL,BR}_inc
purpose     : increment the counter variable coming from the encoder sensor 
parameters  : (none)
return      : (none) 
**************************************************************************/
// increment the counters
void FL_inc() { FL_count++; }
void FR_inc() { FR_count++; }
void BL_inc() { BL_count++; }
void BR_inc() { BR_count++; }

/**************************************************************************
function    : saturate
purpose     : saturate input commands to prevent sending values that are too high or too low
parameters  : 
    * u -  input motor command signal
return 
    * u_sat - saturated input signal 
**************************************************************************/
int saturate(float u, int u_min, int u_max){
    if (u > u_max) { u = u_max; }
    if (u < u_min) { u = u_min; }
    return u;
}

/**************************************************************************
function    : ang2srv
purpose     : convert desired vehicle steering angle [deg] to an PWM value for the arduino "motor" object
parameters  : 
    * ang - input angle [deg]
return       
    * pwm - servo pulse width modulation [pwm] signal 
**************************************************************************/
float ang2srv(float ang){
    return 92.0558 + 1.8194*ang - 0.0104*ang*ang; 
}

/**************************************************************************
function    : motor_map
purpose     : convert desired motor command to an PWM value for the arduino "motor" object
parameters  : 
    * u - motor command [int]
return       
    * u_pwm - motor pulse width modulation [pwm] signal 
**************************************************************************/
float motor_map(float u){
    if(u == 0){ return 90; }
    if(u > 0){ return u + 95} 
    if(u < 0){ return 90;}
}
