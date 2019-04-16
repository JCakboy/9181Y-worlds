#include "main.h"

// Dump ports namespace for ease of use
using namespace ports;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void doubleShot() {
  flywheelMotor->move(127);
  while (flywheelMotor->get_actual_velocity() < 580)
    pros::delay(1);
  indexMotor->move(127);
  pros::delay(45);
  indexMotor->move(0);
  pros::delay(30);
  pid->move(25);
  indexMotor->move(127);
  intakeMotor->move(127);
  pros::delay(300);
  indexMotor->move(0);
  intakeMotor->move(0);
}

void visionAlign() {
  while (flagVision != NULL) {
    // PID values
    double kp = 0.32;
    double kd = 0.42;

    // Prepare variables for decision
    int sigID = LCD::isAutonomousBlue() ? 1 : 2;
    pros::vision_object_s_t sig = flagVision->get_by_sig(0, sigID);
    int middle = util::sign(sig.x_middle_coord);
    double error = middle - 158;
    // If a signature is detected, lock to it. Otherwise, give control back over to the driver
    if (middle > -2000) {
			if (util::abs(error) > 4) {
        double turnPower = error * kp + (12 * util::abs(error) / error) + (kd * (error - visionLastError));
        int leftPower = turnPower;
      	int rightPower = -turnPower;
      	frontLeftDrive->move(leftPower);
      	backLeftDrive->move(leftPower);
      	frontRightDrive->move(rightPower);
      	backRightDrive->move(rightPower);
        visionLastError = error;
      } else
			    break;
  	}
    pros::delay(20);
  }
  visionLastError = 0;
}

void autonomousSkills() {}

void autonomousBlueFlags() {
  // Start the flywheel
  flywheelMotor->move(127);

  // Shake the lift
  liftMotor->move(127);
  pros::delay(150);
  liftMotor->move(-127);
  pros::delay(350);

  // Lock the lift
  liftMotor->move_absolute(269, 100);

  // Grab the platform ball
  intakeMotor->move(100);
  pid->move(16.6);
  liftMotor->move_absolute(51, 100);
  pros::delay(350);
  pid->move(-17);

  // Turn and vision align to the flags
  pid->pivot(110);
  pid->move(4.6);
  intakeMotor->move(0);
  visionAlign();
  liftMotor->move_absolute(269, 100);

  // Shoot for the high and mid flags
  doubleShot();

  // Shutdown the flywheel
  flywheelMotor->move(0);

  // Drive forward and toggle the low flag
  pid->move(8.2);

  // Get in position for next routine
  pid->move(-24.3);
  pid->pivot(-108);

  // Drive forward and intake the ball and angled cap
  liftMotor->move_absolute(190, 100);
  intakeMotor->move(127);
  indexMotor->move(20);
  pid->move(38.75);
  flywheelMotor->move(127);
  indexMotor->move(0);

  // Turn and flip the ground cap
  pid->move(-9.5);
  liftMotor->move_absolute(0, 100);
  pid->pivot(130);

//   pid->move(13);
//   liftMotor->move_absolute(50, 127);
//   pros::delay(100);
//   pid->move(-10);
//   pid->move(12);
//   liftMotor->move_absolute(269, 100);

  // pid->move(6.5);
  // liftMotor->move_absolute(277, 127);
  // pros::delay(210);
  liftMotor->move_absolute(277, 127);
  intakeMotor->move(-100);
  pid->move(15.3);
  intakeMotor->move(0);

  // Turn and fire at the mid flag, and ram into the low cap
  // pid->pivot(-50);
  pid->pivot(-70);
  pid->resetEncoders();
  while (frontLeftDrive->get_position() < 355) {
    pid->driveStraight(127);
    pros::delay(10);
  }
  intakeMotor->move(127);
  indexMotor->move(127);
  pid->resetEncoders();
  while (frontLeftDrive->get_position() < 540) {
    pid->driveStraight(127);
    pros::delay(10);
  }
  pid->driveStraight(0);
}

void autonomousRedFlags() {
  // Start the flywheel
  flywheelMotor->move(127);

  // Shake the lift
  liftMotor->move(127);
  pros::delay(150);
  liftMotor->move(-127);
  pros::delay(350);

  // Lock the lift
  liftMotor->move_absolute(269, 100);

  // Grab the platform ball
  intakeMotor->move(100);
  pid->move(14.8);
  liftMotor->move_absolute(51, 100);
  pros::delay(350);
  pid->move(-14.5);

  // Turn and vision align to the flags
  pid->pivot(-115);
  pid->move(5);
  intakeMotor->move(0);
  visionAlign();
  liftMotor->move_absolute(269, 100);

  // Shoot for the high and mid flags
  doubleShot();

  // Shutdown the flywheel
  flywheelMotor->move(0);

  // Drive forward and toggle the low flag
  pid->move(8.75);

  // Get in position for next routine
  pid->move(-22);
  pid->pivot(105);

  // Drive forward and intake the ball and angled cap
  liftMotor->move_absolute(183, 100);
  intakeMotor->move(127);
  pid->move(38.35);
  flywheelMotor->move(127);
  pros::delay(100);

  // Turn and flip the ground cap
  pid->move(-8);
  liftMotor->move_absolute(0, 100);
  pid->pivot(-130);

//   pid->move(13);
//   liftMotor->move_absolute(50, 127);
//   pros::delay(100);
//   pid->move(-10);
//   pid->move(12);
//   liftMotor->move_absolute(269, 100);
  pid->move(5.9);
  liftMotor->move_absolute(277, 127);
  pros::delay(100);

  // Turn and fire at the mid flag, and ram into the low cap
  pid->pivot(50);
  pid->resetEncoders();
  while (frontLeftDrive->get_position() < 355) {
    pid->driveStraight(127);
    pros::delay(10);
  }
  intakeMotor->move(127);
  indexMotor->move(127);
  pid->resetEncoders();
  while (frontLeftDrive->get_position() < 540) {
    pid->driveStraight(127);
    pros::delay(10);
  }
  pid->driveStraight(0);

  // pid->move(15);
  // intakeMotor->move(127);
  // indexMotor->move(127);
  // pros::delay(300);
  // pid->move(12);
}

void autonomousBlueFar() {

}

void autonomousRedFar() {

}

void autonomousOther(int selected) {

}

void autonomous() {
  switch (selectedAutonomous) {
    case 0:
      autonomousSkills();
      break;
    case 1:
      autonomousBlueFlags();
      break;
    case 2:
      autonomousRedFlags();
      break;
    case 3:
      autonomousBlueFar();
      break;
    case 4:
      autonomousRedFar();
      break;
    default:
      autonomousOther(selectedAutonomous);
      break;
  }
}
