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

int PID::checkPower(int power) {
  //check if power is within the positive value ranges
  if(power > 0) {
    if(power > MAX_POWER) {
      power = MAX_POWER;
    }
    else if(power < MIN_POWER) {
      power = MIN_POWER;
    }
  }
  //check if power is within the negative value ranges
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
  frontRightDrive->move(powerRight);
  backLeftDrive->move(powerLeft);
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
  int power = 20;
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

    //determine power and check to make sure the power is within constraints
    power = (error * kp) + (derivative * kd);
    power = checkPower(power);

    driveStraight(power);
    pros::delay(20);
  }
}
