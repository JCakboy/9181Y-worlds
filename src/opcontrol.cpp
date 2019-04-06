#include "main.h"

// Dump ports namespace for ease of use
using namespace ports;

// Drives the robot based on the given controller
void drive(pros::Controller * controller) {
	int movePower = controller->get_analog(STICK_LEFT_Y);
	int turnPower = controller->get_analog(STICK_LEFT_X);

	// If vision align is requested, take over the turn power
	if (controller->get_digital(BUTTON_L2) && flagVision != NULL) {
		// Prepare variables for decision
		int sigID = LCD::isAutonomousBlue() ? 1 : 2;
		pros::vision_object_s_t sig = flagVision->get_by_sig(0, sigID);
		int middle = util::sign(sig.x_middle_coord);
		int diff = middle - 158;
		// If a signature is detected, lock to it. Otherwise, give control back over to the driver
		if (middle > -2000)
			if (util::abs(diff) > 6)
				turnPower = middle > 158 ?
												diff / 3 + 10:
												diff / 3 - 10;
			else
				turnPower = 0;
		else;
	}

	int leftPower = movePower + turnPower;
	int rightPower = movePower - turnPower;
	frontLeftDrive->move(leftPower);
	backLeftDrive->move(leftPower);
	frontRightDrive->move(rightPower);
	backRightDrive->move(rightPower);
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	// Flag to set when the arm is locked
	bool armLock = false;

	// Flog to set when the flywheel is running
	bool flywheelRunning = false;

	while (true) {
		// Drives the robot with the main controller
		drive(controllerMain);

		// Maps the right joystick to the lift, implementing armLock
		if (controllerMain->get_analog(STICK_RIGHT_Y) == 0 && armLock)
      liftMotor->move_absolute(269, 100);
    else {
      if (armLock) armLock = false;
      liftMotor->move(controllerMain->get_analog(STICK_RIGHT_Y));
    }

		// Lock the lift if A is pressed
		if (controllerMain->get_digital(BUTTON_A))
			armLock = true;

		// Maps the intake motor to the right triggers
		intakeMotor->move(util::limit127((double) controllerMain->get_digital(BUTTON_R1) * 2 * 127 - controllerMain->get_digital(BUTTON_R2) * 127));

		// Maps the index motor forward to the high left trigger and backward to B
		indexMotor->move(util::limit127((double) controllerMain->get_digital(BUTTON_L1) * 2 * 127 - controllerMain->get_digital(BUTTON_B) * 127));

		// Toggles the flywheel power when X is pressed
		if (controllerMain->get_digital_new_press(BUTTON_X)) {
			flywheelRunning = !flywheelRunning;
			flywheelMotor->move(flywheelRunning ? 127 : 0);
		}

		// If down is pressed, reset the lift
		if (controllerMain->get_digital_new_press(BUTTON_DOWN))
			liftMotor->tare_position();

		// Maps the left and right buttons on the controller to the left and right buttons on the Brain LCD
    if (controllerMain->get_digital_new_press(BUTTON_LEFT)) LCD::onLeftButton();
    if (controllerMain->get_digital_new_press(BUTTON_RIGHT)) LCD::onRightButton();

		pros::delay(20);
	}
}
