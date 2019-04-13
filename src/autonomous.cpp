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
  pros::delay(40);
  indexMotor->move(0);
  pid->move(24);
  indexMotor->move(127);
  intakeMotor->move(127);
  pros::delay(300);
  indexMotor->move(0);
  intakeMotor->move(0);
}

void visionAlign() {
  while (true) {
    if (flagVision != NULL) {
      // Prepare variables for decision
		int sigID = LCD::isAutonomousBlue() ? 1 : 2;
		pros::vision_object_s_t sig = flagVision->get_by_sig(0, sigID);
		int middle = util::sign(sig.x_middle_coord);
		int diff = middle - 158;
		// If a signature is detected, lock to it. Otherwise, give control back over to the driver
		if (middle > -2000)
			if (util::abs(diff) > 2) {
				int turnPower = diff / 3 + 10 * util::abs(diff) / diff;
        int leftPower = turnPower;
      	int rightPower = -turnPower;
      	frontLeftDrive->move(leftPower);
      	backLeftDrive->move(leftPower);
      	frontRightDrive->move(rightPower);
      	backRightDrive->move(rightPower);
      } else
				break;
		else;
  	}
  }
}

void autonomous() {
  // doubleShot();
  flywheelMotor->move(127);
  liftMotor->move(127);
  pros::delay(150);
  liftMotor->move(-127);
  pros::delay(500);
  liftMotor->move_absolute(269, 100);
  intakeMotor->move(100);
  pid->move(14.8);
  liftMotor->move_absolute(51, 100);
  pros::delay(350);
  pid->move(-13.5);


  pid->pivot(-110);
  pid->move(6.5);
  intakeMotor->move(0);
  pros::delay(250);
  visionAlign();
  liftMotor->move_absolute(240, 100);
  doubleShot();
  flywheelMotor->move(0);

  pid->move(8);

  pros::delay(50);

  pid->move(-5.5);

  pid->pivot(95);

  intakeMotor->move(127);

  pid->move(4.5);

  pros::delay(250);

  pid->move(-2.5);
}
