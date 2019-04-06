#include "main.h"

// Dump ports namespace for ease of use
using namespace ports;

// Returns the calculated full gear ratio
double PID::getGearRatio() {
  double in = 1;
  double out = 1;
  double wheelDiameter = 4; // in inches
  // Calculates and returns the full gear ratio
  return (360 * out) / (wheelDiameter * PI * in);
}

// Sets the brake mode
void PID::setBrakeMode() {
  frontLeftDrive->set_brake_mode(BRAKE_BRAKE);
  backLeftDrive->set_brake_mode(BRAKE_BRAKE);
  frontRightDrive->set_brake_mode(BRAKE_BRAKE);
  backRightDrive->set_brake_mode(BRAKE_BRAKE);
}

// Resets the motor encoders
void PID::resetEncoders() {
  frontLeftDrive->tare_position();
  backLeftDrive->tare_position();
  frontRightDrive->tare_position();
  backRightDrive->tare_position();
}

// Returns the power given the minimum and maximum power restraints
int PID::checkPower(int power) {
  if (!power);
  else if (util::abs(power) > MAX_POWER) power = power / util::abs(power) * MAX_POWER;
  else if (util::abs(power) < MIN_POWER) power = power / util::abs(power) * MIN_POWER;
  return power;
}

// Ensures the robot drives straight
void PID::driveStraight(int power) {
  double kp = straightkp;
  double master = 0;
  double partner = 0;
  int powerLeft = 0;
  int powerRight = 0;

  // Assign master to the faster moving side
  if (util::abs(frontLeftDrive->get_position()) > util::abs(frontRightDrive->get_position())) {
    master = frontLeftDrive->get_position();
    partner = frontRightDrive->get_position();
  } else if (util::abs(frontLeftDrive->get_position()) < util::abs(frontRightDrive->get_position())) {
    master = frontRightDrive->get_position();
    partner = frontLeftDrive->get_position();
  }

  // Calculate the difference between the wheels from the two sides of the chassis
  double error = master - partner;

  // Decrease the side that is moving faster than the other
  if (util::abs(frontLeftDrive->get_position()) >= util::abs(frontRightDrive->get_position())) {
    powerLeft = power - (error * kp);
    powerRight = power;
  }
  else if (util::abs(frontLeftDrive->get_position())  < util::abs(frontRightDrive->get_position())) {
    powerLeft = power;
    powerRight = power - (error * kp);
  }

  // Ensure that power is within constraints
  powerLeft = checkPower(powerLeft);
  powerRight = checkPower(powerRight);

  // Issue the power to the motors
  frontLeftDrive->move(powerLeft);
  backLeftDrive->move(powerLeft);
  frontRightDrive->move(powerRight);
  backRightDrive->move(powerRight);
}

// Moves the robot the given amount of inches to the desired location
void PID::move(double inches) {
  double kp = movekp;
  double kd = movekd;
  double currentDistance = 0;
  double error = 1;
  double derivative = 0;
  double lastError = 0;
  int power = 0;

  //convert targetDistance from inches to degrees
  double targetDistance = inches * getGearRatio();

  setBrakeMode();
  resetEncoders();

  while (error != 0) {
    currentDistance = (frontRightDrive->get_position() + frontLeftDrive->get_position()) / 2;
    // Calculate proportional error term and the derivative term
    error = targetDistance - currentDistance;
    derivative = error - lastError;
    lastError = error;

    // Determine power and checks if power is within constraints
    power = (error * kp) + (derivative * kd);
    power = checkPower(power);

    // Passes the requested power to the straight drive method
    driveStraight(power);
    pros::delay(20);
  }
}

// Pivots the robot the given amount of degrees
void PID::pivot(double degrees) {
  double kp = pivotkp;
  double kd = pivotkd;
  double currentBearing = gyro1->get_value();
  double error = 1;
  double derivative = 0;
  double lastError = 0;
  int power = 0;

  /* Converts targetBearing to a 10th of a degree.
     Gyro is never reset, so currentBearing is added to targetBearing */
  double targetBearing = (targetBearing * 10.0)  + currentBearing;

  while (error != 0) {
    currentBearing = gyro1->get_value();
    // Calculates difference from targetBearing
    error = targetBearing - currentBearing;
    derivative  = error - lastError;
    lastError = error;

    // Determines power and checks if power is within constraints
    power = (error * kp) + (derivative  * kd);
    power = checkPower(power);

    // Powers drive motors to pivot
    frontLeftDrive->move(power);
    backLeftDrive->move(power);
    frontRightDrive->move(-power);
    backRightDrive->move(-power);

    pros::delay(20);
  }
}
