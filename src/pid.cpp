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

void PID::driveStraight(int power) {
  int kp = 0;
  int master = 0;
  int partner = 0;
  int powerLeft = 0;
  int powerRight = 0;

  if(abs(frontLeftDrive->get_position())  > abs(frontRightDrive->get_position())) {
    master = frontLeftDrive->get_position();
    partner = frontRightDrive->get_position();
  }
  else if(abs(frontLeftDrive->get_position())  < abs(frontRightDrive->get_position())) {
    master = frontRightDrive->get_position();
    partner = frontLeftDrive->get_position();
  }

  int error = master - partner;

  if(abs(frontLeftDrive->get_position()) >= abs(frontRightDrive->get_position())) {
    powerLeft = power - (error * kp);
    powerRight = power;
  }
  else if(abs(frontLeftDrive->get_position())  < abs(frontRightDrive->get_position())) {
    powerLeft = power;
    powerRight = power - (error * kp);
  }

  checkPower(powerLeft);
  checkPower(powerRight);

  frontLeftDrive->move(powerLeft);
  frontRightDrive->move(powerRight);
  backLeftDrive->move(powerLeft);
  backRightDrive->move(powerRight);
}

void PID::distancePID(int targetDistance) {
  int kp = 0;
  int kd = 0;
  int currentDistance = 0;
  int error = 30;
  int derivative = 0;
  int lastError = 0;
  int power = 20;
  targetDistance = targetDistance / wheelCircumference * 360;

  setBrakeMode();
  resetEncoders();

  while(error != 0) {
    currentDistance = (frontRightDrive->get_position() + frontLeftDrive->get_position()) / 2;
    error = targetDistance - currentDistance;
    derivative = error - lastError;
    lastError = error;

    power = (error * kp) + (derivative * kd);
    power = checkPower(power);

    driveStraight(power);
    pros::delay(20);
  }
}
