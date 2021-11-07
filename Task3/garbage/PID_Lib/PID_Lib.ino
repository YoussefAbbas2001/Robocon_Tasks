#include <PID_v1.h>

#define MOTOR_DIR  1  //PB_1
#define MOTOR_PWM  2  //PB_10

double Setpoint ; //Desired Value
double Input;     //Sensor
double Output ;   //Actutor
double Kp=0, Ki=10, Kd=0; 
 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
 
void setup()
{
  Serial.begin(9600);   
  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOT_PWM,OUTPUT);
  Setpoint = 75;
  myPID.SetMode(AUTOMATIC);    //turn on PID
  myPID.SetTunings(Kp, Ki, Kd);// Tunning PID by my Parameter
}
 
void loop()
{
  //Read the value from the light sensor. Analog input : 0 to 1024. We map is to a value from 0 to 255 as it's used for our PWM function.
  Input = map(analogRead(5), 0, 1024, 0, 255);  // photo senor is set on analog pin 5
  //PID calculation
  myPID.Compute();
  //Write the output as calculated by the PID function
  digitalWrite(MOTOR_DIR,HIGH);
  analogWrite(MOTOR_PWM,Output); //LED is set to digital 3 this is a pwm pin. 
  //Send data by serial for plotting 
  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Output);
  Serial.print(" ");  
  Serial.println(Setpoint);
//  delay(100); 
}
