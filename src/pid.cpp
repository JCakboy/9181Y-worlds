#include "main.h"

const int MAX_POWER = 90;
const int MIN_POWER = 20;
using namespace ports;

void setBrakeMode() {
  frontLeftDrive->set_brake_mode(BRAKE_BRAKE);
  backLeftDrive->set_brake_mode(BRAKE_BRAKE);
  frontRightDrive->set_brake_mode(BRAKE_BRAKE);
  backRightDrive->set_brake_mode(BRAKE_BRAKE);
}

void resetEncoders() {
  frontLeftDrive->tare_position();
  backLeftDrive->tare_position();
  frontRightDrive->tare_position();
  backRightDrive->tare_position();
}

int checkIfPowerInConstraints(int power) {
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

void driveStraight(int power, int error) {
  int kp = 0;

  int master = (abs(frontLeftDrive->get_position())  >= abs(frontRightDrive->get_position()) ? frontLeftDrive->get_position() : frontRightDrive->get_position());
}

void distancePID(int targetDistance) {
  int kp = 0;
  int kd = 0;
  int currentDistance = 0;
  int error = 30;
  int derivative = 0;
  int lastError = 0;
  int power = 20;

  setBrakeMode();
  resetEncoders();

  while(error != 0) {
    currentDistance = (frontRightDrive->get_position() + frontLeftDrive->get_position()) / 2;
    error = targetDistance - currentDistance;
    derivative = error - lastError;
    lastError = error;

    power = (error * kp) + (derivative * kd);
    power = checkIfPowerInConstraints(power);

    driveStraight(power, error);
    pros::delay(20);
  }
}
