#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <msgs/IntStamped.h>
#include <Stepper.h>

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

int state_ = -1;

ros::NodeHandle nh;

void tipper(const msgs::IntStamped& msg);
void up();
void down();
//void darthVader();

ros::Subscriber<msgs::IntStamped> sub("arduino_goal", &tipper);
msgs::IntStamped answer_;
ros::Publisher pub("arduino_answer", &answer_);

void tipper(const msgs::IntStamped& msg){
    switch(msg.data){
    case 0:
        up();
        break;

    case 1:
//        darthVader();
        break;

    case 2:
        down();
        break;
    }
}

void up(){
    digitalWrite(EnA, HIGH);
    digitalWrite(EnB, HIGH);

    if(Sensor_Up!=1){
        while(digitalRead(Sensor_Up)!=1){
            StepMotor.step(stepsPerRevolution);
        }
    }

    digitalWrite(EnA, LOW);
    digitalWrite(EnB, LOW);

    state_ = 0;
}

void down(){
    digitalWrite(EnA, HIGH);
    digitalWrite(EnB, HIGH);

    if(Sensor_Down!=1){
      while(digitalRead(Sensor_Down)!=1){
        StepMotor.step(-stepsPerRevolution);
      }
    }

    digitalWrite(EnA, LOW);
    digitalWrite(EnB, LOW);

    state_ = 2;
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
    nh.advertise(pub);
    down();
}

void loop() {
    // put your main code here, to run repeatedly:
    //publish();
    answer_.data = state_;
    answer_.header.stamp = nh.now();
    pub.publish(&answer_);
    nh.spinOnce();
    delay(1000);
}

//void darthVader(){
//    digitalWrite(EnA, HIGH);
//    digitalWrite(EnB, HIGH);

//    //Empirial March
//    StepMotor.setSpeed(20);
//    StepMotor.step(35);
//    delay(60);
//    StepMotor.step(-35);
//    delay(60);
//    StepMotor.step(35);String
//    delay(60);
//    StepMotor.setSpeed(15);
//    StepMotor.step(-20);
//    delay(50);
//    StepMotor.setSpeed(24);
//    StepMotor.step(14);
//    StepMotor.setSpeed(20);
//    StepMotor.step(-35);
//    delay(30);
//    StepMotor.setSpeed(15);
//    StepMotor.step(20);
//    delay(50);
//    StepMotor.setSpeed(24);
//    StepMotor.step(-14);
//    StepMotor.setSpeed(20);
//    StepMotor.step(65);
//    delay(100);
//    StepMotor.setSpeed(30);
//    StepMotor.step(-50);
//    delay(60);
//    StepMotor.step(50);
//    delay(60);
//    StepMotor.step(-50);
//    delay(60);
//    StepMotor.setSpeed(32);
//    StepMotor.step(40);
//    delay(50);
//    StepMotor.setSpeed(24);
//    StepMotor.step(14);
//    StepMotor.setSpeed(19);
//    StepMotor.step(-30);
//    delay(60);
//    StepMotor.setSpeed(15);
//    StepMotor.step(20);
//    delay(50);
//    StepMotor.setSpeed(24);
//    StepMotor.step(-14);
//    StepMotor.setSpeed(20);
//    StepMotor.step(70);

//    digitalWrite(EnA, LOW);
//    digitalWrite(EnB, LOW);

//    state_ = 1;
//}
