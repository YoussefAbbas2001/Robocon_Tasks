/*
Author : Youssef Abbas 
Date   : 6/11/2021
About  : PID control DC Motor
TO     : ROBOCON Traning Task3
*/
//#include <ros.h>
//#include <std_msgs/Float64.h>

#define SIGNAL_A PA_0  //PA_0
#define SIGNAL_B PA_1  //PA_1
#define PWM      PB_10 //PB_10
#define DIR      PB_1  //PB_1
#define PWM_LIM  65536 //Limit of PWM signal
#define PPRC     512   //Pulse Per Revolution Cycle
#define MIN      60    //minute

//ros::NodeHandle nh;
//float target;
//void callback( const std_msgs::Float64& msg){ target = msg.data;}
//ros::Subscriber<std_msgs::Float64> sub("drive_motor", &callback );

double  pwm      ;         //PWM signal  of MOTOR
double  control  ;         //Control Signal Out of PID control 
double  set_point= 0    ;  //target speed
double  integral = 0    ;  //Summation of error Added to control signal to overcome Steady-State Error
double  diff     = 0    ;  //Derivative term
double  kp =  2  ;         //Proportional Constant
double  ki =  1  ;         //Intergral Constant
double  kd =  1  ;         //Derivative Constant
double  error=0  ;         //Error Signal (E = T - P)
double  prev_e=0 ;         //previous error (to compute Derivaitve term)
double  velocity ;         //Current Velocity of Motor
double  distance ;         //Distance cutted by Motor in Meter
double  delta_t  ;         //Change time = (current time) - (previous time)
double  delta_e  ;         //Change Error= (Current error)- (previous error)
double  prev_t=0 ;         //Previous time used to computer delta of time 
double  curr_t=0 ;         //Current time usef to compute  delta of time
double  counter=0;         //Counter of Encoder
double  prev_c=0 ;         //Previous Count used to compute delta of distance
double  curr_c=0 ;         //Current Count used to compute delta of distance
double  delta_c=0;         //Change in Distance = (Current Count) - (Previous Count)

HardwareSerial Serial3(PB11, PB10);

void setup() 
{
  Serial3.begin(9600);
//  nh.initNode();                   //intialize node to subscribe
//  nh.subscribe(sub);               //subscribe data
  pinMode(SIGNAL_A, INPUT_PULLUP);   //Encoder A
  pinMode(SIGNAL_B, INPUT_PULLUP);   //Encoder B
  pinMode(DIR,OUTPUT);               //Dirction of DC MOTOR
  pinMode(PWM,OUTPUT);               //Speed of DC MOTOR


  set_point = 0  ;            //get set point from published data  
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_B), ISR_B, CHANGE);

//  (nh.getHardware()->setPort(&Serial1));
//  (nh.getHardware()->setBaud(115200));
//  nh.initNode();
//  nh.subscribe(target);
}

void loop()
{
  calc();                                          //get (Velocity) and (delta time) 

  if (control<0)                                   //When Control is negative (Exceed Target) 
  {
      digitalWrite(DIR , HIGH)
  }
  else if (control>0) 
  {
      digitalWrite(DIR,LOW)
  }   
  pwm = abs(control);                              //Absolute value of Control
  if(pwm > PWM_LIM)                                //if Control signal exceed max PWM
     pwm = PWM_LIM;                                //Maximum PWM signal in Arduino UNO
  analogWrite(PWM,pwm);                            // Motor speed

  Serial3.print("rpm: ");
  Serial3.println(error);
  Serial3.print("error :");
  Serial3.println(pwm );
  Serial3.print("Speed : ");
  Serial3.print(velocity);
  Serial3.println(",");
  delay(50);

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
  curr_t     = millis();
  curr_c     = counter;
  delta_t    =(curr_t - prev_t)         ;          //Change in time  (ms)
  delta_c    =(curr_c - prev_c)         ;          //Change in count (pulse)
  velocity   =(delta_c)/(delta_t) *1000 ;          //curr_speed      (pulse/s)  
  velocity   =(velocity/PPRC)*MIN       ;          //curr_speed      (RPM)
  
  error     = set_point - velocity;                //error = <Target Velocity> - <Current Velocity>    
  delta_e   = error-prev_e;                        //Change in Error
  integral  = integral + ki*error*delta_t;         //Interation of Error to overcome Steady-state error
  diff      = delta_e/delta_t ;                    //Derivative Term to overcome Damping
  control   = kp*error + integral + kd*diff;       //Control Equation
//  control   = 100*control;                       //Map Control Signal to get better Response

//Privous Values
  prev_e     = error;                           
  prev_c     = curr_c;         
  prev_t     = curr_t;

}
