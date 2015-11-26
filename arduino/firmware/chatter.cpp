#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Stepper.h>
//#include <msgs/IntStamped.h>
const int stepsPerRevolution = 200;
boolean ActiveIdleSwitch = false; 
int Button = 5;
int Sensor_Down = 6;
int Sensor_Up = 7;
int EnA = 8;
int In1 = 9;
int In2 = 10;
int In3 = 11;
int In4 = 12;
int EnB = 13;
Stepper StepMotor(stepsPerRevolution, In2, In1, In3, In4);
ros::NodeHandle nh;
//msgs::IntStamped automode_msg;
//ros::Publisher pub_button("/fmPlan/automode", &automode_msg);


void tipper(const std_msgs::String& unload_msg){
      
      digitalWrite(EnA, HIGH);
      digitalWrite(EnB, HIGH);
      
      
      
      if(Sensor_Up!=1){
        while(digitalRead(Sensor_Up)!=1){
            StepMotor.step(stepsPerRevolution);
          }
      }
      delay(2000);
      if(Sensor_Down!=1){
        while(digitalRead(Sensor_Down)!=1){
          StepMotor.step(-stepsPerRevolution);
        }
      }
    
      digitalWrite(EnA, LOW);
      digitalWrite(EnB, LOW);
      return;
  }
ros::Subscriber<std_msgs::String> sub("unload", &tipper );

void darthVader(){
      digitalWrite(EnA, HIGH);
      digitalWrite(EnB, HIGH);

      //Empirial March
      StepMotor.setSpeed(20);
      StepMotor.step(35);
      delay(60);
      StepMotor.step(-35);
      delay(60);
      StepMotor.step(35);
      delay(60);
      StepMotor.setSpeed(15);
      StepMotor.step(-20);
      delay(50);
      StepMotor.setSpeed(24);
      StepMotor.step(14);
      StepMotor.setSpeed(20);
      StepMotor.step(-35);
      delay(30);
      StepMotor.setSpeed(15);
      StepMotor.step(20);
      delay(50);
      StepMotor.setSpeed(24);
      StepMotor.step(-14);
      StepMotor.setSpeed(20);
      StepMotor.step(65);
      delay(100);
      StepMotor.setSpeed(30);
      StepMotor.step(-50);
      delay(60);
      StepMotor.step(50);
      delay(60);
      StepMotor.step(-50);
      delay(60);
      StepMotor.setSpeed(32);
      StepMotor.step(40);
      delay(50);
      StepMotor.setSpeed(24);
      StepMotor.step(14);
      StepMotor.setSpeed(19);
      StepMotor.step(-30);
      delay(60);
      StepMotor.setSpeed(15);
      StepMotor.step(20);
      delay(50);
      StepMotor.setSpeed(24);
      StepMotor.step(-14);
      StepMotor.setSpeed(20);
      StepMotor.step(70);
      digitalWrite(EnA, LOW);
      digitalWrite(EnB, LOW);
      return;
  }

void setup() {
  // put your setup code here, to run once:
  StepMotor.setSpeed(150);
  pinMode(Button, INPUT_PULLUP);
  pinMode(Sensor_Down, INPUT_PULLUP);
  pinMode(Sensor_Up, INPUT_PULLUP);
  pinMode(EnA, OUTPUT);
  pinMode(EnB, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  //nh.advertise(pub_button);
}
/*
void publish(){
	automode_msg.header.stamp=nh.now();
	automode_msg.data=digitalRead(Button);
	pub_button.publish(&automode_msg);
}*/

void loop() {
  // put your main code here, to run repeatedly:
   //publish();
   nh.spinOnce();
   delay(1000);
}
