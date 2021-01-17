#include "encoders.h"
#include "motors.h"
#include "pid.h"
#define DELAY 100

#define LINE_LEFT_PIN A2 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN A4 //Pin for the right line sensor


//Pin definitions for motor
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN  9
#define R_DIR_PIN 15
#define BAUD_RATE = 115200;

float KP = 1.14; //Proportional gain for position controller
float KD = -0.56; //Derivative gain for position controller
float KI = 0; //Integral gain for position controller

int leftSpeed = 0;
int rightSpeed = 0;

PID Position_Left(KP, KD, KI);
PID Position_Right(KP, KD, KI);
PID Zone4_Home_Volt_Position_Calc(KP, KD, KI);

bool walk_straight = true;
bool line_following = false;
bool turn_for_return = false;
bool walk_straight_to_home = false;
bool allow_rotation = true;

int Calibrated_White; // global var to store calibrated white value

int encoder_counts_for_home_after_rotation = 0; //after rotation encoder counts
float calc_home_encoder_counts = 0; //calculated distance to return home

Motor left_motor;
Motor right_motor;

float demand = 5000; //Initial target encoder count

void setupMotorPins()
{
  left_motor.setPins(L_PWM_PIN, L_DIR_PIN);
  right_motor.setPins(R_PWM_PIN, R_DIR_PIN);
}

void setup()
{
  setupMotorPins();
  // These two function set up the pin
  // change interrupts for the encoders.
  // If you want to know more, find them
  // at the end of this file.
  setupEncoder0();
  setupEncoder1();

  //setup the timer to get the current time values
  setupTimer3();
  
  pinMode(LINE_LEFT_PIN, INPUT);
  pinMode(LINE_RIGHT_PIN, INPUT);
  pinMode(LINE_CENTRE_PIN, INPUT);
 
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);


  // set led to light after calibration and stopping
  pinMode(13, OUTPUT);

  // set buzzer to start when stopping after line following
  pinMode(6, OUTPUT);

//Calibrating the white value, used to stop at the end

  int Left_Sensor_Reading = analogRead(LINE_LEFT_PIN);
  int Center_Sensor_Reading = analogRead(LINE_CENTRE_PIN);
  int Right_Sensor_Reading = analogRead(LINE_RIGHT_PIN);
  int White_Error_Margin = +20;
  Calibrated_White = max(max(Left_Sensor_Reading, Right_Sensor_Reading), Center_Sensor_Reading)+White_Error_Margin;

  Serial.begin( 9600 );
}

void going_straight_zone1() {
  float left_change_volt = Position_Left.update(demand, count_e0);
  float right_change_volt = Position_Right.update(demand, count_e1);
  int calc_change_volt_without_pid = left_change_volt - right_change_volt;

  if(left_change_volt < 18 && right_change_volt < 18){
    //stop motors for very small voltage values
    right_motor.setSpeed(0);
    left_motor.setSpeed(0);
    
  }
  else if (calc_change_volt_without_pid > 0) { 
    //left goes faster. increase right
    right_motor.setSpeed(right_change_volt + calc_change_volt_without_pid);
    left_motor.setSpeed(left_change_volt);
  }
  else {
    //right goes faster, decrease left
    left_motor.setSpeed(left_change_volt - calc_change_volt_without_pid);
    right_motor.setSpeed(right_change_volt);
  }
}


  //line following function
void follow_line_zone2() {
   
  int Left_Sensor_Reading = analogRead(LINE_LEFT_PIN);
  int Center_Sensor_Reading = analogRead(LINE_CENTRE_PIN);
  int Right_Sensor_Reading = analogRead(LINE_RIGHT_PIN);

  int Calc_Reading_Difference = Left_Sensor_Reading - Right_Sensor_Reading;

  Serial.print("White value");
  Serial.println(Calibrated_White);
  if (Left_Sensor_Reading <= Calibrated_White && Right_Sensor_Reading <= Calibrated_White && Center_Sensor_Reading <= Calibrated_White)
  {
    stop_walking();
    analogWrite(6,200);
    delay(1000);
    analogWrite(6,0);
    delay(3000);
    line_following = false;
    turn_for_return = true;
  }

  else if (Calc_Reading_Difference <= -200 )
  {
    turn_right();
  }

  else if (Calc_Reading_Difference >= 200)
  {
    turn_left();
  }

  else
  {
    move_forward();
  }
}

void stop_walking()
{
  //setting of directions not required for stopping
  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);
}

void turn_right()
{
  digitalWrite(L_DIR_PIN, LOW);
  analogWrite(L_PWM_PIN, 20);
  digitalWrite(R_DIR_PIN, HIGH);
  analogWrite(R_PWM_PIN, 20);
}

void turn_left()
{
  digitalWrite(L_DIR_PIN, HIGH);
  analogWrite(L_PWM_PIN, 20);
  digitalWrite(R_DIR_PIN, LOW);
  analogWrite(R_PWM_PIN, 20);
}

void move_forward()
{
  digitalWrite(L_DIR_PIN, LOW);
  analogWrite(L_PWM_PIN, 20);
  digitalWrite(R_DIR_PIN, LOW);
  analogWrite(R_PWM_PIN, 20);
}

// rotate to go home in zone 4
void rotate_home_zone4() {

//  digitalWrite(L_DIR_PIN, HIGH);
//  analogWrite(L_PWM_PIN, 16);
//  digitalWrite(R_DIR_PIN, LOW);
//  analogWrite(R_PWM_PIN, 16);
//  delay(1630);
//   // after rotation delay
//  turn_for_return = false;
//  walk_straight_to_home = true;
//  
//   count_e0 = 0;
//   count_e1 = 0;
  int before_rotation_encoder_count;
  float encoder_counts_in_rotation;

 
  if (allow_rotation) {
    calc_home_encoder_counts = calc_home_displacement();
    allow_rotation = false;
    before_rotation_encoder_count = count_e0;
    float calc_angle =calc_home_angle();
    encoder_counts_in_rotation =  calc_angle + before_rotation_encoder_count;
  }

  digitalWrite(13, HIGH); // light the led when reaches end

  float left_volt;
  if (encoder_counts_in_rotation > count_e0) { // turn anti-clockwise
    left_volt = Position_Left.update(encoder_counts_in_rotation, count_e0);
  }
  else {
    left_volt = Position_Left.update(count_e0, encoder_counts_in_rotation);
  }

  left_motor.setSpeed(-left_volt);
  right_motor.setSpeed(left_volt);

  if (left_volt < 18) {
    turn_for_return = false;
    count_e0 = 0;
    count_e1 = 0;
    encoder_counts_for_home_after_rotation = count_e0;
    walk_straight_to_home = true;
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    delay(3000); // after rotation delay
  }

}
// Zone 4 PID based going home
void going_home_zone4() {
  float left_change_volt = Position_Left.update(demand, count_e0);
  float right_change_volt = Position_Right.update(demand, count_e1);
  float cal_change_volt = Zone4_Home_Volt_Position_Calc.update(count_e0, count_e1);

  if(left_change_volt < 18 && right_change_volt < 18){
    right_motor.setSpeed(0);
    left_motor.setSpeed(0);
  }
  else if (cal_change_volt > 0) {
    right_motor.setSpeed(right_change_volt + cal_change_volt);
    left_motor.setSpeed(left_change_volt);
  }
  else {
    left_motor.setSpeed(left_change_volt - cal_change_volt);
    right_motor.setSpeed(right_change_volt);
  }
}

bool CheckOnLine() {
  int Left_Sensor_Reading = analogRead(LINE_LEFT_PIN);
  int Center_Sensor_Reading = analogRead(LINE_CENTRE_PIN);
  int Right_Sensor_Reading = analogRead(LINE_RIGHT_PIN);
  bool return_value;
  if (Center_Sensor_Reading > 600) {
    return_value = true;
  }
  else {
    return_value = false;
  }
  return return_value;
}

//integration code to merge all 4 zones
void loop()
{
  if (walk_straight) {
    if (!CheckOnLine()) {
      going_straight_zone1();
    }
    else {
      walk_straight = false;
      line_following = true;
    }
  }

  if (line_following) {
    follow_line_zone2();
  }

  if (turn_for_return) {
    rotate_home_zone4();
  }

  if (walk_straight_to_home) {
//    int Encoder_Counts_Error_Margin = 1390;
//    demand = encoder_counts_for_home_after_rotation + calc_home_encoder_counts - Encoder_Counts_Error_Margin;
      demand=9830;
    going_home_zone4();
  }
  delay(DELAY); //adding loop delay to check sensors properly
}
