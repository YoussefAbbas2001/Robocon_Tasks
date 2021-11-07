/*
Author : Youssef Abbas 
Date   : 6/11/2021
About  : PID control DC Motor
TO     : ROBOCON Traning Task3
*/
#include <ros.h>
#include <std_msgs/Float64.h>

#define SIGNAL_A PA_0  //PA_0
#define SIGNAL_B PA_1  //PA_1
#define PWM      PB_10 //PB_10
#define DIR      PB_1  //PB_1
#define PWM_LIM  65536 //Limit of PWM signal
#define PPRC     512   //Pulse Per Revolution Cycle
#define MIN      60    //minute
#define TAR_RED  0.1   //reduis of Wheel  in meter (Assumed)

ros::NodeHandle nh;
float target;
void callback( const std_msgs::Float64& msg){ target = msg.data;}
ros::Subscriber<std_msgs::Float64> sub("drive_motor", &callback );

bool     dir     ;         //direction signal of MOTOR
uint16_t pwm     ;         //PWM signal  of MOTOR
float   control  ;         //Control Signal Out of PID control 
float   set_point= 0    ;  //target speed
float   integral = 0    ;  //Summation of error Added to control signal to overcome Steady-State Error
float   diff     ;         //Derivative term
float   kp = 10  ;         //Proportional Constant
float   ki =  5  ;         //Intergral Constant
float   kd =  1  ;         //Derivative Constant
float   error=0  ;         //Error Signal (E = T - P)
float   prev_e   ;         //previous error (to compute Derivaitve term)
float   velocity ;         //Current Velocity of Motor
float   distance ;         //Distance cutted by Motor in Meter
float   delta_t  ;         //Change time = (current time) - (previous time)
float   delta_e  ;         //Change Error= (Current error)- (previous error)
float   prev_t=0 ;         //Previous time used to computer delta of time 
float   curr_t=0 ;         //Current time usef to compute  delta of time
uint32_t counter=0;        //Counter of Encoder
uint32_t prev_c=0;         //Previous Count used to compute delta of distance
uint32_t curr_c=0;         //Current Count used to compute delta of distance
uint32_t delta_c ;         //Change in Distance = (Current Count) - (Previous Count)

void setup() 
{
  nh.initNode();            //intialize node to subscribe
  nh.subscribe(sub);        //subscribe data
  pinMode(DIR,OUTPUT);      //Dirction of DC MOTOR
  pinMode(PWM,OUTPUT);      //Speed of DC MOTOR
  pinMode(SIGNAL_A, INPUT); //Encoder A
  pinMode(SIGNAL_B, INPUT); //Encoder B

  
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_B, CHANGE);
}

void loop()
{
  set_point = target     ;                         //get set point from published data
  get_speed();                                     //get (Velocity) and (delta time) 
  prev_e    = error;                               //Previous Error to compute  Derivative Term
  error     = set_point - velocity;                //error = <Target Velocity> - <Current Velocity>    
  delta_e   = error-prev_e;                        //Change in Error
  integral  = integral + error*delta_t;            //Interation of Error to overcome Steady-state error
  diff      = delta_e/delta_t ;                    //Derivative Term to overcome Damping
  control   = kp*error + ki*integral + kd*diff;    //Control Equation

  dir = 1;                                         //Direction of Motor forward (under target)
  if (control<0)                                   //When Control is negative (Exceed Target) 
      dir = -1;                                    //Direction of Motor Reverse (over target)
  pwm = (uint16_t) fabs(control);                  //Absolute value of Control
  if(pwm > PWM_LIM)                                //if Control signal exceed max PWM
     pwm = PWM_LIM;                                //Maximum PWM signal in Arduino UNO
     
  setMotor(dir,pwm);                               //Put Process Signal in Planet
  nh.spinOnce();
}



void setMotor(int dir, int pwmVal)
{
  analogWrite(PWM,pwmVal); // Motor speed
}

void ISR_A()
{
  if (digitalRead(SIGNAL_A) != digitalRead(SIGNAL_B))
    counter++  ;
  else
    counter--  ;
}

void ISR_B()
{
  if (digitalRead(SIGNAL_A) == digitalRead(SIGNAL_B))
    counter++   ;
  else
    counter--  ;
}

void get_speed()
{
  prev_t     = curr_t;
  prev_c     = curr_c;         
  curr_t     = millis();
  curr_c     = counter;
  
  delta_t    =(curr_t - prev_t)/1000;   //Change in time  (s)
  delta_c    =(curr_c - prev_c)     ;   //Change in count (pulse)
  velocity   =(delta_c)/(delta_t)   ;   //curr_speed      (pulse/s)  
  velocity   = velocity/(PPRC*MIN)  ;   //curr_speed      (RPM)
}

void get_distance()
{
  //Unused for now
  distance =(float)((counter/PPRC)*2*PI*TAR_RED);    //Distance in Meter
}
