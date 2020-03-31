
//this code for interfacing with motor and to get feedback from motor and imu
//subscribes pwm signal and direction and control direction and speed of motor
//take position feedback from encoders and count ticks of both counters AB channels and publishes tick as int topic
//take imu feedback and publishes acc_x . acc_y . acc_z

//motor - A right
//motor - B left

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <Encoder.h>
ros::NodeHandle nh;
Encoder myEnc(2, 20);
int pwm_right;
int pwm_left;
int direction_flag = 0;



void messageCb_direction( const std_msgs::Int32& msg_direction){
  direction_flag = msg_direction.data;
  if (direction_flag == 1){
    clockwise();
  }
  else if (direction_flag == 2){
    counterClockwise();
  }
  else {
    brake();
  }
}
void messageCb_right( const std_msgs::Int32& msg_right){
  pwm_right = msg_right.data;
  analogWrite(3, pwm_right);
	
}
void messageCb_left( const std_msgs::Int32& msg_left){
  pwm_left = msg_left.data;
  analogWrite(9, pwm_left);
}
ros::Subscriber<std_msgs::Int32> sub_direction("direction", messageCb_direction );
ros::Subscriber<std_msgs::Int32> sub_pwm_right("pwm_right", messageCb_right );
ros::Subscriber<std_msgs::Int32> sub_pwm_left("pwm_left", messageCb_left );
//////std_msgs::Int32 acc_x;
//////ros::Publisher pub_acc_x("accelerometer_x", &acc_x);
//////std_msgs::Int32 acc_y;
//////ros::Publisher pub_acc_y("accelerometer_y", &acc_y);
//////std_msgs::Int32 acc_z;
//////ros::Publisher pub_acc_z("accelerometer_z", &acc_z);

sensor_msgs::Imu imu_data;
ros::Publisher imu_pub("imu", &imu_data);

std_msgs::Int32 lt_count_A;
ros::Publisher pub_lt_A("left_Count_A", &lt_count_A);

int enA = 9;
int in1 = 8;
int in2 = 7;
// Motor B connections // left
int enB = 3;
int in3 = 5;
int in4 = 4;
//IMU pins
const int xpin = A0;
const int ypin = A1;
const int zpin = A2;
//pos enc pins
const int left_A_pin = 2;

long left_A_lastState = -999;     // previous state of the button
int leftEncA = 0;
void setup()
{
  nh.initNode();
  ////nh.advertise(pub_acc_x);
  ////nh.advertise(pub_acc_y);
  ////nh.advertise(pub_acc_z);
  nh.advertise(pub_lt_A);
  nh.advertise(imu_pub);
  nh.subscribe(sub_direction);
  nh.subscribe(sub_pwm_right);
  nh.subscribe(sub_pwm_left);
  
	// Set all the motor control pins to outputs
  pinMode(left_A_pin, INPUT);
  //speed control pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  //motor A pins - right
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //motor B pins - left
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

}
 
// encoder event for the interrupt call
// need to check the logic

void loop()
{
  //str_msg.data = hello;
  //chatter.publish( &str_msg );
  //nh.loginfo (pwm_right.data);
  count_pos_enc();
  nh.spinOnce();
  imu();
 

}
void count_pos_enc() {
  nh.loginfo ("1");
  
  long left_A_currentState = myEnc.read();
  if(left_A_currentState != left_A_lastState){
     nh.loginfo ("2");
     left_A_lastState = left_A_currentState;
  } 
  lt_count_A.data = leftEncA;
  pub_lt_A.publish(&lt_count_A);

}
void clockwise() {

	// Turn on motor A & B
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	
}
void counterClockwise() {
	
	// Now change motor directions
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);

}

void brake() {

	// Turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}
//accelerometer//when you rotate the device the screen changes its orientation
//Finding the roll and pitch of the device using analog voltages of accelerometer module
void imu() {
  int x = analogRead(xpin); 
  delay(50);
  
  int y = analogRead(ypin);
  delay(50);
  
  int z = analogRead(zpin);
  delay(50);
  ////acc_x.data=x-322;
  /////acc_y.data=y-320;
  ///////acc_z.data=z-263;
  
    //need to check wt is this value
  //Serial.print(x-322);
  //Serial.print(x);
  //Serial.print("\t");
  
  //Serial.print(y-320);
  //Serial.print(y);
  //Serial.print("\t");
  
  //Serial.print(z-263);
  //Serial.print(z);
  //Serial.print("\n"); 
  
  ////pub_acc_x.publish(&acc_x);
  /////pub_acc_y.publish(&acc_y);
  /////pub_acc_z.publish(&acc_z);
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp=nh.now();
  imu_msg.header.frame_id = "/hokuyo_link";
  //need to edit processing and pub of imu data.. not crct
  float linacc_conversion = .00025*9.8;
  imu_msg.linear_acceleration.x = (x)*linacc_conversion;
  imu_msg.linear_acceleration.y = (y)*linacc_conversion;
  imu_msg.linear_acceleration.z = (z)*linacc_conversion;

  imu_pub.publish(&imu_msg);
}
