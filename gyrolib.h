/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
|
| Project: GyroLib_4_0
| Creators: Reza Torbati, Andrew Zhang, Michael Bartlett, Aaron Pierce
| Date: Spring 2020
| Description: functions to turn and drive straight using the gyro
| Contact: Reza Torbati - kofcrotmgiv@gmail.com    Andrew Zhang - infinitepolygons@gmail.com
|
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/
#include <kipr/botball.h>
#include <pthread.h>
double bias;          													//Variable to hold the calibration value
double ninetyDegrees; 													//How many KIPR degrees are in a regular degrees.
int right_motor, left_motor;  											//Motor Ports
double right_motor_coefficient = 1, left_motor_coefficient = 1; 	//Ratio of power of the wheels to correct for variations in motor power
int in_degrees = 0;  													//Checks if the user is in regular degrees or KIPR degrees. 0 is KIPR degrees. Would be a boolean if C had those
double absolute_theta = 0;  											//The theta that thetaTracker updates to
int mode = 0; 															//The mode with 0 being relative and 1 being absolute
int timeout = 120000;  													//Stops functions after a certain amount of time in milliseconds, defaults to 120 seconds
char axis = 'z';  														//The axis that the program reads from. Defaults to z
thread thetaTracker;  													//Thread to track the absolute theta
//initializes the ports for the motors. Sets unit to KIPR degrees
void declare_motors(int lmotor, int rmotor)
{
  right_motor = rmotor;
  left_motor = lmotor;
  in_degrees = 0;
}
//allows you to manually input degrees without giving the ports for the left and right wheels. Great for creates. Sets unit from KIPR degrees to normal degrees
void declare_degrees(double degree)
{
  ninetyDegrees = degree;
  in_degrees = 1;
}
//initializes the ports for the motors and also allows you to input degrees without having to do calibrate_degrees(). Sets unit from KIPR degrees to normal degrees
void declare_motors_and_degrees(int lmotor, int rmotor, double degree)
{
  right_motor = rmotor;
  left_motor = lmotor;
  ninetyDegrees = degree;
  in_degrees = 1;
}
//should mainly be used as a private function to get gyro readings from your axis of choice
int get_gyro_reading()
{
  if(axis == 'z')
      return gyro_z();
  else if(axis == 'x')
      return gyro_x();
  else if(axis == 'y')
      return gyro_y();
  else
  {
      printf("ERROR: The gyro axis must either be x, y, or z\n");
      exit(2);
  }
}
//Filters the gyro reading for potentially more accuracy. Accepts a number that corresponds to different filters
int get_filtered_gyro_reading(int filter)
{
  switch(filter)
  {
          //Filter 0 is no filter
      case 0:
          return get_gyro_reading();
          break;
          //Filter 1 is a floor filter. It floors each number to the nearest 10
      case 1:
          return (int)(get_gyro_reading() * 10) / 10;
          break;
          //Default is no filter
      default:
          return get_gyro_reading();
  }
}
//call this function to determine how many KIPR degrees are in a regular degree. Sets the unit from KIPR degrees to normal degrees.
//The robot must already be calibrated, be at a complete stop before pressing the right button, do not turn to fast and KEEP THE ROBOT FLAT WHEN TURNING
void calibrate_degrees()
{
  double theta = 0;
  printf("Put the robot flat on the table, make sure it is at a complete stop, and press the right button\n");
  while(right_button() == 0);
  printf("Now turn the robot 360degrees while keeping it flat and then press the left button\n");
  double t0 = seconds();
  while(left_button() == 0)
  {
      theta += (get_gyro_reading() - bias) * (seconds() - t0);
      t0 = seconds();
  }
  ninetyDegrees = abs(theta / 4.0);
  printf("90 degrees is %f KIPR degrees\n",ninetyDegrees);
  in_degrees = 1;
}
//Calibrates the bias. Stops the robot for sleep_time. Make sleep_time just large enough so that it at a COMPLETE stop before finding the bias.
void calibrate_gyro_advanced(int calibration_time, int sample_delay)
{
  ao();
  //Calculate the average reading of the still gyroscope
  double start_time = seconds();
  double sum = 0;
  int i = 0;
  while(seconds() - start_time < calibration_time/1000)
  {
      sum += get_gyro_reading();
      i++;
      msleep(sample_delay);
  }
  //Divide how much the gyroscope has drifted by time to get the drift per second
  bias = sum/i;
  printf("New Bias: %f\n", bias);//prints out your bias. COMMENT THIS LINE OUT IF YOU DON'T WANT BIAS READINGS PRINTED OUT
}
//Abbreviated calibrate_gyro_advanced()
void cga(int calibration_time, int sample_delay)
{
  calibrate_gyro_advanced(calibration_time, sample_delay);
}
//Simplified calibrate_gyro_advanced()
void calibrate_gyro()
{
  calibrate_gyro_advanced(5000, 5);
}
//Abbreviated calibrate_gyro()
void cg()
{
  calibrate_gyro();
}
//This function is created solely for start_theta_tracker to use. Think of this as a private function
void update_theta()
{
  double t0 = seconds();
  while(1)
  {
      absolute_theta += (get_gyro_reading() - bias) * (seconds() - t0);
      t0 = seconds();
  }
}
//Begins the thread to track theta
void start_theta_tracker()
{
  thetaTracker = thread_create(update_theta);
  thread_start(thetaTracker);
}
//Stops the thread to track theta
void stop_theta_tracker()
{
  thread_destroy(thetaTracker);
}
//One function that can be used for all the gyroscope set up at the start of a program
void initialize_gyro(int lmotor, int rmotor, int degree)
{
  declare_motors_and_degrees(lmotor, rmotor, degree);
  cg();
  start_theta_tracker();
}
//Abbrevaited initialize_gyro()
void ig(int lmotor, int rmotor, int degree)
{
  initialize_gyro(lmotor, rmotor, degree);
}

//Funtions to set and get global variables
//returns the theta that the theta tracker is getting
char get_axis()
{
return axis;
}
//must be called to set the axis from z to x or y
void set_axis(char a)
{
  if(a == 'x' || a == 'y' || a == 'z')
      axis = a;
  else
  {
      printf("ERROR: The gyro axis must either be x, y, or z\n");
      exit(2);
  }
}
double get_absolute_theta()
{
  if(in_degrees)
      return absolute_theta / (ninetyDegrees / 90);
  return absolute_theta;
}
void set_absolute_theta(double value)
{
  if(in_degrees)
      absolute_theta = value * (ninetyDegrees / 90);
  else
  {
      absolute_theta = value;
  }
}
int get_mode()
{
  return mode;
}
void set_mode(int value)
{
  if(value == 0 || value == 1)
  {
      mode = value;
  }
}
int get_timeout()
{
  return timeout;
}
void set_timeout(int value)
{
  timeout = value;
}
double get_bias()
{
  return bias;
}
void set_right_motor_coefficient(double rmc)
{
  right_motor_coefficient = rmc;
}
void set_left_motor_coefficient(double lmc)
{
  left_motor_coefficient = lmc;
}
double get_right_motor_coefficient()
{
  return right_motor_coefficient;
}
double get_left_motor_coefficient()
{
  return left_motor_coefficient;
}

//turns the robot to reach a desired theta. Make sure that your robot is turing in the correct direction or it will never stop. If you are expecting this function to work consistantly then don't take your turns too fast.
//This is considerably worse than the new turn_with_gyro for pivot turns but is more consistent from robot to robot and allows the robot to arc.
void arc_with_gyro(int left_wheel_speed, int right_wheel_speed, double target_theta)
{
  //Set up the start time to break if function goes too long
  double start_time = seconds();
  //Changes between kipr degrees and degrees
  if(in_degrees)
  {
      target_theta *= (ninetyDegrees / 90);
  }
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  //Sets the motor speeds
  mav(right_motor, right_wheel_speed * right_motor_coefficient);
  mav(left_motor, left_wheel_speed * left_motor_coefficient);
  if(target_theta > absolute_theta)//positive turns left
  {
      //Keeps the motors going until it reaches the desired angle
      while(target_theta > absolute_theta)
      {
          //Stop the function if the overshoot time is exceeded
          if((seconds() - start_time) * 1000 > timeout)
          {
              printf("Function Timed out. Error: %f\n", target_theta - absolute_theta);
              mav(right_motor, 0);
              mav(left_motor, 0);
              break;
          }
      }
  }
  else
  {
      //Keeps the motors going until it reaches the desired angle
      while(target_theta < absolute_theta)
      {
          //Stop the function if the overshoot time is exceeded
          if((seconds() - start_time) * 1000 > timeout)
          {
              printf("Function Timed out. Error: %f\n", target_theta - absolute_theta);
              mav(right_motor, 0);
              mav(left_motor, 0);
              break;
          }
      }
  }
  //Stops the motors at the end of the turn
  mav(right_motor, 0);
  mav(left_motor, 0);
}
//Abbreviated arc_with_gyro()
void awg(int lspeed, int rspeed, double target_theta)
{
  arc_with_gyro(lspeed, rspeed, target_theta);
}
//The correction constant is a positive number (usually ~.01) that has to be adjusted based on the robot and turn. The higher it is the faster the robot
//turns but if it is too high the the robot will jump around and destroy motors but if it is too low then the robot will never reach its target.
//overshoot is how long in ms you want the robot to continue to turn after it reaches its target to account for overshooting the target.
//targetTheta needs to be positive for left turns and negative for right turns.
void turn_with_gyro_overshoot(double correctionConstant, double target_theta, double overshoot)
{
  //Set up the start time to break if function goes too long
  double start_time = seconds();
  //Converts to kipr degrees
  if(in_degrees)
  {
      target_theta *= (ninetyDegrees / 90);
  }
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  //Converts from milliseconds to seconds
  overshoot /= 1000;
  //Boolean checks if it has overshot yet
  int overshot = 0;
  double overshotTime = 0;
  while(seconds() < overshotTime + overshoot || overshot == 0)
  {
      if(target_theta >= absolute_theta)
      {
          mav(left_motor, -(target_theta - absolute_theta) * correctionConstant);
          mav(right_motor,(target_theta - absolute_theta) * correctionConstant);
          if(absolute_theta >= target_theta && !overshot)
          {
              overshot = 1;
              overshotTime = seconds();
          }
      }
      else
      {
          mav(left_motor, (absolute_theta - target_theta) * correctionConstant);
          mav(right_motor, -(absolute_theta - target_theta) * correctionConstant);
          if(absolute_theta < target_theta && !overshot)
          {
              overshot = 1;
              overshotTime = seconds();
          }
      }
      //Stop the function if the overshoot time is exceeded
      if((seconds() - start_time) * 1000 > timeout)
      {
          printf("Function Timed out. Error: %f\n", target_theta - absolute_theta);
          mav(left_motor, 0);
          mav(right_motor, 0);
          break;
      }
  }
  //Stops the motors at the end of the turn
  mav(left_motor, 0);
  mav(right_motor, 0);
}
//Abbreviated turn_with_gyro_overshoot()
void twgo(double correctionConstant, double targetTheta, double overshoot)
{
  turn_with_gyro_overshoot(correctionConstant, targetTheta, overshoot);
}
//Turns to a angle using PID controls
void turn_with_gyro_advanced(double target_theta, double speed_limit, double pk, double ik, double dk)
{
  //Converts to kipr degrees
  if(in_degrees)
  {
      target_theta *= (ninetyDegrees / 90);
  }
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  double start_time = seconds();
  double t0 = seconds();
  double error = target_theta - absolute_theta;
  double p, i, d, pid;
  int previous_error = error;
  int max_speed = speed_limit;
  //Keep turning until the error reaches 0
  while(abs(error) > 0)
  {
      //Update the P, I, and D Values
      p = error;
      i += error * (seconds() - t0);
      d = (error - previous_error) / (seconds() - t0);
      t0 = seconds();
      pid = p*pk + i*ik + d*dk;
      //Cap the pid value to the motor limitations
      if(pid > max_speed)
      {
          pid = max_speed;
      }
      if(pid < -max_speed)
      {
          pid = -max_speed;
      }
      //Move the robot
      mav(right_motor, pid  * right_motor_coefficient);
      mav(left_motor, -pid  * left_motor_coefficient);
      //Update theta and error
      error = target_theta - absolute_theta;
      //Break out of the loop if timeout is exceeded
      if((seconds() - start_time) * 1000 > timeout)
      {
          printf("Function Timed Out. Error: %f\n", error);
          mav(left_motor, 0);
          mav(right_motor, 0);
          break;
      }
  }
  //Stops the motors at the end of the turn
  mav(left_motor, 0);
  mav(right_motor, 0);
}
//Abbreviated turn_with_gyro_advanced()
void twga(double target_theta, double speed_limit, double pk, double ik, double dk)
{
  turn_with_gyro_advanced(target_theta, speed_limit, pk, ik, dk);
}
//Simplified turn_with_gyro_advanced()
void turn_with_gyro(double target_theta)
{
  turn_with_gyro_advanced(target_theta, 1500, 12, 0, 0);
}
//Abbreviated turn_with_gyro()
void twg(double target_theta)
{
  turn_with_gyro(target_theta);
}
//Drives straight forward or backwards. The closer speed is to 0 the faster it will correct itself and the more consistent it will be but just do not go at max speed and it'll be fine.
//Time is in ms.
//Correction speed is the speed that the robot corrects itself at the end. The robot will not correct at the end if the speed is <= 0
//If correction speed is between 0 and 1 then will run turn_with_gyro instead of arc_with_gyro to correct
void drive_with_gyro_advanced(int speed, int time, double pk, int correction)
{
  double target_theta = absolute_theta;
  double start_time = seconds();
  double error = absolute_theta - target_theta;
  while(seconds() - start_time < (time / 1000.0))
  {
      mav(right_motor, (speed * right_motor_coefficient) - (pk * error));
      mav(left_motor, (speed * left_motor_coefficient) + (pk * error));
      error = absolute_theta - target_theta;
  }
  //Turn at the end to be certain that it ends at the same angle it started at if correction does not equal 0
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  if(correction != 0)
  {
      turn_with_gyro(target_theta);
  }
  //Stop the motors at the end of the drive
  mav(right_motor, 0);
  mav(left_motor, 0);
}
//Abbreviated drive_with_gyro_advanced()
void dwga(int speed, int time, double pk, double correction_speed)
{
  drive_with_gyro_advanced(speed, time, pk, correction_speed);
}
//Simplified drive_with_gyro_advanced()
void drive_with_gyro(int speed, int time)
{
  drive_with_gyro_advanced(speed, time, 12, 0);
}
//Abbreviated drive_with_gyro()
void dwg(int speed, int time)
{
  drive_with_gyro(speed, time);
}
//Drives until an analog input
int drive_until_analog_advanced(int speed, int port, int target_value, double pk, double max_time)
{
  //Set up the initial variables
  double start_time = seconds();
  double target_theta = absolute_theta;
  double error = absolute_theta - target_theta;
  //Check the initial value
  int initial_value = analog(port);
  //Drive until the value goes above the target value if it started below the target value
  if(initial_value < target_value)
  {
      while(analog(port) < target_value)
      {
          mav(right_motor, (speed * right_motor_coefficient) - (pk * error));
          mav(left_motor, (speed * left_motor_coefficient) + (pk * error));
          error = absolute_theta - target_theta;
          //Break out of the loop if timeout is exceeded
          if((seconds() - start_time) * 1000 > max_time)
          {
              printf("Function Timed Out. Error: %f\n", error);
              mav(left_motor, 0);
              mav(right_motor, 0);
              return 0;
          }
      }
  }
  //Drive until the sensor value goes below the target value if it started above the target value
  else
  {
      while(analog(port) > target_value)
      {
          mav(right_motor, speed - (pk * error));
          mav(left_motor, speed + (pk * error));
          error = absolute_theta - target_theta;
          //Break out of the loop if timeout is exceeded
          if((seconds() - start_time) * 1000 > max_time)
          {
              printf("Function Timed Out. Error: %f\n", error);
              mav(left_motor, 0);
              mav(right_motor, 0);
              return 0;
              break;
          }
      }
  }
  //Stop the motors at the end of the drive
  mav(right_motor, 0);
  mav(left_motor, 0);
    return 1;
}
//Abreviated drive_until_analog_advanced()
int duaa(int speed, int port, int target_value, double pk, double max_time)
{
  return drive_until_analog_advanced(speed, port, target_value, pk, max_time);
}
//Simplified drive_until_analog_advanced()
int drive_until_analog(int speed, int port, int target_value)
{
  return drive_until_analog_advanced(speed, port, target_value, 12, 120000);
}
//Abbreviated drive_until_analog()
int dua(int speed, int port, int target_value)
{
  return drive_until_analog(speed, port, target_value);
}

void drive_until_analog_advanced_compound(int speed, int port1, int port2, int target_value, double pk, double max_time)
{
  //Set up the initial variables
  double start_time = seconds();
  double target_theta = absolute_theta;
  double error = absolute_theta - target_theta;
  //Check the initial value
  int initial_value = analog(port1) + analog(port2);
  //Drive until the value goes above the target value if it started below the target value
  if(initial_value < target_value)
  {
      while(analog(port1) + analog(port2) < target_value)
      {
          mav(right_motor, (speed * right_motor_coefficient) - (pk * error));
          mav(left_motor, (speed * left_motor_coefficient) + (pk * error));
          error = absolute_theta - target_theta;
          //Break out of the loop if timeout is exceeded
          if((seconds() - start_time) * 1000 > max_time)
          {
              printf("Function Timed Out. Error: %f\n", error);
              mav(left_motor, 0);
              mav(right_motor, 0);
              break;
          }
      }
  }
  //Drive until the sensor value goes below the target value if it started above the target value
  else
  {
      while(analog(port1) + analog(port2) > target_value)
      {
          mav(right_motor, (speed * right_motor_coefficient) - (pk * error));
          mav(left_motor, (speed * left_motor_coefficient) + (pk * error));
          error = absolute_theta - target_theta;
          //Break out of the loop if timeout is exceeded
          if((seconds() - start_time) * 1000 > max_time)
          {
              printf("Function Timed Out. Error: %f\n", error);
              mav(left_motor, 0);
              mav(right_motor, 0);
              break;
          }
      }
  }
  //Stop the motors at the end of the drive
  mav(right_motor, 0);
  mav(left_motor, 0);
}
//Abbreviated and simplified drive_until_analog_compound_advanced()
void duac(int speed, int port1, int port2, int target_value)
{
    drive_until_analog_advanced_compound(speed, port1, port2, target_value, 12, 120000);
}

//Drives until a digital input
void drive_until_digital_advanced(int speed, int port, double pk, double max_time)
{
  //Set up the initial variables
  double start_time = seconds();
  double target_theta = absolute_theta;
  double error = absolute_theta - target_theta;
  //Check the initial value
  int initial_value = digital(port);
  //Drive until the value in the digital sensor changes
  while(digital(port) == initial_value)
  {
      mav(right_motor, (speed * right_motor_coefficient) - (pk * error));
      mav(left_motor, (speed * left_motor_coefficient) + (pk * error));
      error = absolute_theta - target_theta;
      //Break out of the loop if timeout is exceeded
      if((seconds() - start_time) * 1000 > max_time)
      {
          printf("Function Timed Out. Error: %f\n", error);
          mav(left_motor, 0);
          mav(right_motor, 0);
          break;
      }
  }
  //Stop the motors at the end of the drive
  mav(right_motor, 0);
  mav(left_motor, 0);
}
//Abbreviated drive_until_digital_advanced()
void duda(int speed, int port, double pk, double max_time)
{
  drive_until_digital_advanced(speed, port, pk, max_time);
}
//Simplified drive_until_digital_advanced()
void drive_until_digital(int speed, int port)
{
  drive_until_digital_advanced(speed, port, 12, 120000);
}
//Abbreviated drive_until_digital()
void dud(int speed, int port)
{
  drive_until_digital(speed, port);
}
//Drives with gyro a certain distance
void drive_with_gyro_distance_advanced(int speed, int target_distance, int motor, double pk, int correction)
{
  double target_theta = absolute_theta;
  int initial_distance = gmpc(motor);
  double error = absolute_theta - target_theta;
  //Drive until the desired distance is reached
  while(abs(gmpc(motor) - initial_distance) < target_distance)
  {
      mav(right_motor, (speed * right_motor_coefficient) - (pk * error));
      mav(left_motor, (speed * left_motor_coefficient) + (pk * error));
      error = absolute_theta - target_theta;
  }
  //Turn at the end to be certain that it ends at the same angle it started at if correction does not equal 0
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  if(correction != 0)
  {
      turn_with_gyro(target_theta);
  }
  //Stop the motors at the end of the drive
  mav(right_motor, 0);
  mav(left_motor, 0);
}
//Abbreviated drive_with_gyro_distance_advanced()
void dwgda(int speed, int target_distance, int motor, double pk, int correction)
{
  drive_with_gyro_distance_advanced(speed, target_distance, motor, pk, correction);
}
//Simplified drive_with_gyro_distance_advanced()
void drive_with_gyro_distance(int speed, int target_distance, int motor)
{
  drive_with_gyro_distance_advanced(speed, target_distance, motor, 12, 0);
}
//Abbreviated drive_with_gyro_distance()
void dwgd(int speed, int target_distance, int motor)
{
  drive_with_gyro_distance(speed, target_distance, motor);
}

//Finds the ratio between the motor speeds to allow for straighter driving
void calibrate_ticks_advanced(int speed, int time)
{
    int l_start_pos = gmpc(left_motor);
    int r_start_pos = gmpc(right_motor);
    dwg(speed, time);
    int l_dist = gmpc(left_motor) - l_start_pos;
    int r_dist = gmpc(right_motor) - r_start_pos;
    //If left wheel went further
    if(l_dist > r_dist)
    {
        printf("Right Motor Coefficient: %f", (double)r_dist/l_dist);   
        right_motor_coefficient = (double)r_dist/l_dist;
    }
    else
    {
        printf("Left Motor Coefficient: %f", (double)l_dist/r_dist);
        left_motor_coefficient = (double)l_dist/r_dist;
    }
}
//Abbreviated calibrate_ticks_advanced()
void cta(int speed, int time)
{
    calibrate_ticks_advanced(speed, time);
}
//Simplified calibrate_ticks_advanced
void calibrate_ticks()
{
 	calibrate_ticks_advanced(1000, 5000);   
}
//Abbreviated calibrate_ticks()
void ct()
{
 	calibrate_ticks();   
}

//CREATE EXCLUSIVE FUNCTIONS BELOW
//Stops the create and calibrates. Exact same as calibrarte_gyro_advanced and should only be used if the create was previously moving and you are too lazy to stop it yourself
void create_calibrate_gyro_advanced(int sleep_time, int sample_size)
{
  create_stop();
  msleep(sleep_time); //to make sure that everything is actually stopped. The sleep time needs to be just high enough to ensure the robot is at a complete stop
  //Calculate the how much the gyroscope thinks it moves
  double initial_time = seconds();
  double t0 = seconds();
  double sum = 0;
  int i = 0;
  while(i < sample_size)
  {
      sum += get_gyro_reading() * (seconds() - t0);
      t0 = seconds();
      i++;
  }
  //Divide how much the gyroscope has drifted by time to get the drift per second
  bias = (sum / (seconds() - initial_time));
  printf("New Bias: %f\n", bias);//prints out your bias. COMMENT THIS LINE OUT IF YOU DON'T WANT BIAS READINGS PRINTED OUT
}
//Abbreviated create_calibrate_gyro_advanced()
void ccga(int sleep_time, int sample_size)
{
  create_calibrate_gyro_advanced(sleep_time, sample_size);
}
//Simplified create_calibrate_gyro_advanced
void create_calibrate_gyro()
{
  create_calibrate_gyro_advanced(0, 5000);
}
//Abbreviated create_calibrate_gyro()
void ccg()
{
  create_calibrate_gyro();
}
//Turns the wheels at given speeds until the desired angle is reached
void create_arc_with_gyro(int left_wheel_speed, int right_wheel_speed, double target_theta)
{
  //Set up the start time to break if function goes too long
  double start_time = seconds();
  //Changes between kipr degrees and degrees
  if(in_degrees)
  {
      target_theta *= (ninetyDegrees / 90);
  }
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  //Sets the motor speeds
  create_drive_direct(right_wheel_speed, left_wheel_speed);
  if(target_theta > absolute_theta)//positive turns left
  {
      //Keeps the motors going until it reaches the desired angle
      while(target_theta > absolute_theta)
      {
          //Stop the function if the overshoot time is exceeded
          if((seconds() - start_time) * 1000 > timeout)
          {
              printf("Function Timed out. Error: %f\n", target_theta - absolute_theta);
              create_stop();
              break;
          }
      }
  }
  else
  {
      //Keeps the motors going until it reaches the desired angle
      while(target_theta < absolute_theta)
      {
          //Stop the function if the overshoot time is exceeded
          if((seconds() - start_time) * 1000 > timeout)
          {
              printf("Function Timed out. Error: %f\n", target_theta - absolute_theta);
              create_stop();
              break;
          }
      }
  }
  //Stops the create at the end of the turn
  create_stop();
}
//Abbrevaite create_arc_with_gyro()
void cawg(int left_wheel_speed, int right_wheel_speed, double target_theta)
{
  create_arc_with_gyro(left_wheel_speed, right_wheel_speed, target_theta);
}
//The correction constant is a positive number (usually ~.01) that has to be adjusted based on the robot and turn. The higher it is the faster the robot
//turns but if it is too high the the robot will jump around and destroy motors but if it is too low then the robot will never reach its target.
//overshoot is how long in ms you want the robot to continue to turn after it reaches its target to account for overshooting the target.
//targetTheta needs to be positive for left turns and negative for right turns.
void create_turn_with_gyro_overshoot(double correctionConstant, double target_theta, double overshoot)
{
  //Set up the start time to break if function goes too long
  double start_time = seconds();
  //Converts to kipr degrees
  if(in_degrees)
  {
      target_theta *= (ninetyDegrees / 90);
  }
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  //Converts from milliseconds to seconds
  overshoot /= 1000;
  //Boolean checks if it has overshot yet
  int overshot = 0;
  double overshotTime = 0;
  while(seconds() < overshotTime + overshoot || overshot == 0)
  {
      if(target_theta >= absolute_theta)
      {
          create_drive_direct(-(target_theta - absolute_theta) * correctionConstant, (target_theta - absolute_theta) * correctionConstant);
          if(absolute_theta >= target_theta && !overshot)
          {
              overshot = 1;
              overshotTime = seconds();
          }
      }
      else
      {
          create_drive_direct((absolute_theta - target_theta) * correctionConstant, -(absolute_theta - target_theta) * correctionConstant);
          if(absolute_theta < target_theta && !overshot)
          {
              overshot = 1;
              overshotTime = seconds();
          }
      }
      //Stop the function if the overshoot time is exceeded
      if((seconds() - start_time) * 1000 > timeout)
      {
          printf("Function Timed out. Error: %f\n", target_theta - absolute_theta);
          create_stop();
          break;
      }
  }
  //Stops the motors at the end of the turn
  create_stop();
}
//Abbreviated create_turn_with_gyro_overshoot()
void ctwgo(double correctionConstant, double targetTheta, double overshoot)
{
  create_turn_with_gyro_overshoot(correctionConstant, targetTheta, overshoot);
}
//Turns to a angle using PID controls
void create_turn_with_gyro_advanced(double target_theta, double speed_limit, double pk, double ik, double dk)
{
  //Converts to kipr degrees
  if(in_degrees)
  {
      target_theta *= (ninetyDegrees / 90);
  }
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  double start_time = seconds();
  double t0 = seconds();
  double error = target_theta - absolute_theta;
  double p, i, d, pid;
  int previous_error = error;
  int max_speed = speed_limit;
  //Keep turning until the error reaches 0
  while(abs(error) > 0)
  {
      //Update the P, I, and D Values
      p = error;
      i += error * (seconds() - t0);
      d = (error - previous_error) / (seconds() - t0);
      t0 = seconds();
      pid = p*pk + i*ik + d*dk;
      //Cap the pid value to the motor limitations
      if(pid > max_speed)
      {
          pid = max_speed;
      }
      if(pid < -max_speed)
      {
          pid = -max_speed;
      }
      //Move the robot
      create_drive_direct(-pid, pid);
      //Update theta and error
      error = target_theta - absolute_theta;
      //Break out of the loop if timeout is exceeded
      if((seconds() - start_time) * 1000 > timeout)
      {
          printf("Function Timed Out. Error: %f\n", error);
          create_stop();
          break;
      }
  }
  //Stops the motors at the end of the turn
  create_stop();
}
//Abbreviated create_turn_with_gyro_advanced()
void ctwga(double target_theta, double speed_limit, double pk, double ik, double dk)
{
  create_turn_with_gyro_advanced(target_theta, speed_limit, pk, ik, dk);
}
//Simplified create_turn_with_gyro_advanced()
void create_turn_with_gyro(double target_theta)
{
  create_turn_with_gyro_advanced(target_theta, 500, 10, 0, 0);
}
//Abbreviated create_turn_with_gyro()
void ctwg(double target_theta)
{
  create_turn_with_gyro(target_theta);
}
//Drives for a given time at a given speed while trying to maintain starting angle
void create_drive_with_gyro_advanced(int speed, int time, double pk, int correction)
{
  double target_theta = absolute_theta;
  double start_time = seconds();
  double error = absolute_theta - target_theta;
  while(seconds() - start_time < (time / 1000.0))
  {
      create_drive_direct(speed + (pk * error), speed - (pk * error));
      error = absolute_theta - target_theta;
  }
  //Turn at the end to be certain that it ends at the same angle it started at if correction does not equal 0
  //Change between absolute and relative theta
  if(mode == 0)
  {
      target_theta += absolute_theta;
  }
  if(correction != 0)
  {
      turn_with_gyro(target_theta);
  }
  //Stop the motors at the end of the drive
  create_stop();
}
//Abbreviated create_drive_with_gyro_advanced()
void cdwga(int speed, int time, double pk, double correction_speed)
{
  create_drive_with_gyro_advanced(speed, time, pk, correction_speed);
}
//Simplified create_drive_with_gyro_advanced()
void create_drive_with_gyro(int speed, int time)
{
  create_drive_with_gyro_advanced(speed, time, 10, 0);
}
//Abbreviated create_drive_with_gyro()
void cdwg(int speed, int time)
{
  create_drive_with_gyro(speed, time);
}
