#include "main.h"

using namespace ports;

void PID::setBrakeMode() {
  frontLeftDrive->set_brake_mode(BRAKE_BRAKE);
  backLeftDrive->set_brake_mode(BRAKE_BRAKE);
  frontRightDrive->set_brake_mode(BRAKE_BRAKE);
  backRightDrive->set_brake_mode(BRAKE_BRAKE);
}

void PID::resetEncoders() {
  frontLeftDrive->tare_position();
  backLeftDrive->tare_position();
  frontRightDrive->tare_position();
  backRightDrive->tare_position();
}

//check if power is within max/min constraints
int PID::checkPower(int power) {
  if(power > 0) {
    if(power > MAX_POWER) {
      power = MAX_POWER;
    }
    else if(power < MIN_POWER) {
      power = MIN_POWER;
    }
  }
  else if(power < 0) {
    if(power < -MAX_POWER) {
      power = -MAX_POWER;
    }
    else if(power > -MIN_POWER) {
      power = -MIN_POWER;
    }
  }
  return power;
}

//makes sure robot drives straight
void PID::driveStraight(int power) {
  int kp = 0;
  int master = 0;
  int partner = 0;
  int powerLeft = 0;
  int powerRight = 0;

  //make master the side that is moving more than the other side
  if(abs(frontLeftDrive->get_position())  > abs(frontRightDrive->get_position())) {
    master = frontLeftDrive->get_position();
    partner = frontRightDrive->get_position();
  }
  else if(abs(frontLeftDrive->get_position())  < abs(frontRightDrive->get_position())) {
    master = frontRightDrive->get_position();
    partner = frontLeftDrive->get_position();
  }
  //calculate the difference between the wheels from the two sides of the chassis
  int error = master - partner;

  //decrease the side that is moving faster than the other
  if(abs(frontLeftDrive->get_position()) >= abs(frontRightDrive->get_position())) {
    powerLeft = power - (error * kp);
    powerRight = power;
  }
  else if(abs(frontLeftDrive->get_position())  < abs(frontRightDrive->get_position())) {
    powerLeft = power;
    powerRight = power - (error * kp);
  }

  //check to make sure power is within constraints
  powerLeft = checkPower(powerLeft);
  powerRight = checkPower(powerRight);

  frontLeftDrive->move(powerLeft);
  backLeftDrive->move(powerLeft);
  frontRightDrive->move(powerRight);
  backRightDrive->move(powerRight);
}

//makes sure robot moves to desired location
void PID::distancePID(int targetDistance) {
  int kp = 0;
  int kd = 0;
  int currentDistance = 0;
  int error = 30;
  int derivative = 0;
  int lastError = 0;
  int power = 0;
  //convert targetDistance from inches to degrees
  targetDistance = targetDistance / wheelCircumference * 360;

  setBrakeMode();
  resetEncoders();

  while(error != 0) {
    currentDistance = (frontRightDrive->get_position() + frontLeftDrive->get_position()) / 2;
    //calculate proportional error term and the derivative
    error = targetDistance - currentDistance;
    derivative = error - lastError;
    lastError = error;

    //determine power and checks if power is within constraints
    power = (error * kp) + (derivative * kd);
    power = checkPower(power);

    driveStraight(power);
    pros::delay(20);
  }
}

//PID to make sure the robot pivots to the correct degree
void PID::pivotPID(int targetBearing) {
  int kp = 0;
  int kd = 0;
  int currentBearing = gyro->get_value();
  int error = 30;
  int derivative = 0;
  int lastError = 0;
  int power = 0;

  /*converts targetBearing to a 10th of a degree.
    gyro is never reset, so currentBearing is added to targetBearing*/
  targetBearing = (targetBearing * 10)  + currentBearing;

  while(error != 0) {
    currentBearing = gyro->get_value();
    //calculates difference from targetBearing
    error = targetBearing - currentBearing;
    derivative  = error - lastError;
    lastError = error;

    //determines power and checks if power is within constraints
    power = (error * kp) + (derivative  * kd);
    power = checkPower(power);

    //powers drive motors to pivot
    frontLeftDrive->move(power);
    backLeftDrive->move(power);
    frontRightDrive->move(-power);
    backRightDrive->move(-power);

    pros::delay(20);
  }
}
