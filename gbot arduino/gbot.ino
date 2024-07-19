// AGV Machine - Vinay Lanka

#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

int updatenh = 0;

#define LOOPTIME 10

Motor right(32, 33, 27, 14);  // Modify pin numbers as per your setup
Motor left(34, 35, 25, 26);   // Modify pin numbers as per your setup

volatile long encoder0Pos = 0;    // Encoder 1
volatile long encoder1Pos = 0;    // Encoder 2

double left_kp = 800.0, left_ki = 400.0, left_kd = 10.0;             // Modify for optimal performance
double right_kp = 800.0, right_ki = 400.0, right_kd = 10.0;

float demandx = 0;
float demandz = 0;

double demand_speed_left;
double demand_speed_right;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);
geometry_msgs::Vector3Stamped speed_msg;                                // Create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          // Create a publisher to ROS topic "speed" using the "speed_msg" type
std_msgs::Int16 left_wheel_msg;
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);
std_msgs::Int16 right_wheel_msg;
ros::Publisher right_wheel_pub("lwheel", &right_wheel_msg);

double pos_act_left = 0;
double pos_act_right = 0;

double speed_act_left = 0;                    // Actual speed for left wheel in m/s
double speed_act_right = 0;                   // Command speed for left wheel in m/s 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);                  // Prepare to publish speed in ROS topic
  nh.advertise(left_wheel_pub);
  nh.advertise(right_wheel_pub);

  ledcSetup(0, 3000, 8);  // Configure PWM channels for ESP32
  ledcSetup(1, 3000, 8);
  ledcAttachPin(right.plus, 0);
  ledcAttachPin(right.minus, 1);
  ledcSetup(2, 3000, 8);
  ledcSetup(3, 3000, 8);
  ledcAttachPin(left.plus, 2);
  ledcAttachPin(left.minus, 3);

  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);

  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left.en_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_b), change_right_b, CHANGE);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME) {
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz * 0.1375); //Wheel separation 275mm = 0.275m, 0.75/2 = 0.1375
    demand_speed_right = demandx + (demandz * 0.1375);

    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;

    pos_act_left = encoder0Pos;
    pos_act_right = encoder1Pos;

    speed_act_left = encoder0Diff / 350;
    speed_act_right = encoder1Diff / 350;

    encoder0Error = (demand_speed_left * 350) - encoder0Diff; // 35000 ticks in 1m = 350 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right * 350) - encoder1Diff;

    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;

    left_setpoint = demand_speed_left * 350;  // Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right * 350;

    left_input = encoder0Diff;  // Input to PID controller is the current difference
    right_input = encoder1Diff;

    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);
  }
  publishSpeed(LOOPTIME);
  if(updatenh>10){
    nh.spinOnce();
    updatenh=0;
  }else{
    updatenh++;
  }
  
}

// Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      // Timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    // Left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   // Right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;       // Looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
}

void publishPos(double time){
  left_wheel_msg.data = pos_act_left;
  right_wheel_msg.data = pos_act_right;
  left_wheel_pub.publish(&left_wheel_msg);
  right_wheel_pub.publish(&right_wheel_msg);
}

// ************** Encoders interrupts **************

// ************** Encoder 1 *********************

void change_left_a() {
  if (digitalRead(left.en_a) == HIGH) {
    if (digitalRead(left.en_b) == LOW) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  } else {
    if (digitalRead(left.en_b) == HIGH) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  }
}

void change_left_b() {
  if (digitalRead(left.en_b) == HIGH) {
    if (digitalRead(left.en_a) == HIGH) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  } else {
    if (digitalRead(left.en_a) == LOW) {
      encoder0Pos = encoder0Pos + 1;  // CW
    } else {
      encoder0Pos = encoder0Pos - 1;  // CCW
    }
  }
}

// ************** Encoder 2 *********************

void change_right_a() {
  if (digitalRead(right.en_a) == HIGH) {
    if (digitalRead(right.en_b) == LOW) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  } else {
    if (digitalRead(right.en_b) == HIGH) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  }
}

void change_right_b() {
  if (digitalRead(right.en_b) == HIGH) {
    if (digitalRead(right.en_a) == HIGH) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  } else {
    if (digitalRead(right.en_a) == LOW) {
      encoder1Pos = encoder1Pos - 1;  // CW
    } else {
      encoder1Pos = encoder1Pos + 1;  // CCW
    }
  }
}
