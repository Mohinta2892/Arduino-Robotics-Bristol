#ifndef _Kinematics
#define _Kinematics_h
#define M_PI 3.14159265358979323846  /* pi */


const int WHEEL_SEPARATION = 125;
const float COUNTS_PER_MM = 0.15;
const float MM_PER_COUNT = 6.666667;
const float RADIAN_TO_ENCODER = 1440 / (2 * M_PI); //multiply by this to convert radians to encoder counts

class Kinematics
{
  public:
    //Public variables and methods go here
    float get_x();
    float get_y();
    float get_rotation_angle();
    float set_x(float curr_x);
    float set_y(float curr_y);
    float set_rotation_angle(float curr_rotation_angle);
    void update(int left_encoder_cnt, int right_encoder_cnt);
    float Kinematics::calc_home_angle(float current_angle);
    float Kinematics::calc_home_displacement();

  private:
    //Private variables and methods go here
    float x_coord = 0;
    float y_coord = 0;
    float rotation_angle = 0;
    int last_left_encoder_cnt = 0;
    int last_right_encoder_cnt = 0;
    float delta_x_coord = 0;
    float delta_y_coord = 0;
};

float Kinematics::get_x() {
  return x_coord;
}

float Kinematics::set_x(float curr_x) {
  x_coord = curr_x;
}

float Kinematics::get_y() {
  return y_coord;
}


float Kinematics::set_y(float curr_y) {
  y_coord = curr_y;
}

float Kinematics::get_rotation_angle() {
  return rotation_angle;
}

float Kinematics::set_rotation_angle(float curr_rotation_angle) {
  rotation_angle = curr_rotation_angle;
}
// calc home displacement
float Kinematics::calc_home_displacement() {
  delta_x_coord = 0 - x_coord;
  delta_y_coord = 0 - y_coord;
  return sqrt((delta_x_coord * delta_x_coord) + (delta_y_coord * delta_y_coord)); // in encoder counts NOT mm
}

//Kinematics update function
void Kinematics::update(int left_encoder_cnt, int right_encoder_cnt) {
  int change_in_left = left_encoder_cnt - last_left_encoder_cnt;
  int change_in_right = right_encoder_cnt - last_right_encoder_cnt;
  float d = (change_in_left + change_in_right / 2);

  last_left_encoder_cnt = left_encoder_cnt;
  last_right_encoder_cnt = right_encoder_cnt;

  rotation_angle = rotation_angle + ((change_in_left * COUNTS_PER_MM) - (change_in_right * COUNTS_PER_MM)) / WHEEL_SEPARATION;

  x_coord = x_coord + d * cos(rotation_angle);
  y_coord = y_coord + d * sin(rotation_angle);

}

//Calculating rotation angle after stopping in line following

float Kinematics::calc_home_angle(float current_angle) {
  delta_x_coord = 0 - x_coord;
  delta_y_coord = 0 - y_coord;

  current_angle = abs(current_angle) + (M_PI / 2);

  float target_theta = atan2(delta_y_coord, delta_x_coord);
  float how_much_to_rotate = target_theta - current_angle;
  float angular_error_margin = 0.175;

  //deciding on clockwise and anticlockwise rotation depending where it stops
  if (how_much_to_rotate < -M_PI) {
    how_much_to_rotate = (2 * M_PI) + how_much_to_rotate ; // clockwise?
  }
  if (how_much_to_rotate > M_PI) {
    how_much_to_rotate = - ((2 * M_PI) - how_much_to_rotate); //anti-clockwise?
  }

  return (how_much_to_rotate - angular_error_margin) * RADIAN_TO_ENCODER ; //in encoder counts
}


#endif
