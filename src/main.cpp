#include "main.h"
// ...existing code...
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */


const double pi = 3.1415926535;

class Drivetrain {
private:
	pros::MotorGroup left_mg = {1, -4, -5};
	pros::MotorGroup right_mg = {-2, 3, 6};
	pros::Imu inertial_sensor = 7;
	pros::Motor intake = 8;
	pros::Motor outtake = 9;
	pros::ADIAnalogOut intakeArm = pros::ADIAnalogOut('A');
	pros::ADIAnalogOut outtakeArm = pros::ADIAnalogOut('B');
	const int intakeSpeed = 30;
	const int outSpeed = 30;
	const double wheel_diameter = 3.25;
	const double ticks_per_rev = 600.0;
	bool intakeState = false;
	bool outtakeState = false;

public:
	enum class OpControlMode { LEFT_ARCADE, RIGHT_ARCADE, SPLIT_ARCADE, TANK };
	OpControlMode opcontrol_mode = OpControlMode::SPLIT_ARCADE;

	void moveOutakeArm() {
		outtakeArm.set_value(!outtakeState);
		outtakeState = !outtakeState;
	}

	void moveIntakeArm() {
		intakeArm.set_value(!intakeState);
		intakeState = !intakeState;
	}
	

	void startRunningIntake(int velocity) {
		intake.move(velocity);
	}
	void startRunningOuttake(int velocity) {
		outtake.move(velocity);
	}

	void stopIntake() {
		intake.brake();
	}

	void stopOuttake() {
		outtake.brake();
	}

	void setOpcontrolMode(OpControlMode mode) {
		opcontrol_mode = mode;
	}

	OpControlMode getOpcontrolMode() const {
		return opcontrol_mode;
	}

	// Move forward a given distance (in inches)
	void moveForward(double inches, int velocity = 100) {
		double revs = inches / (pi * wheel_diameter);
		int ticks = static_cast<int>(revs * ticks_per_rev);

		left_mg.tare_position();
		right_mg.tare_position();

		left_mg.move_absolute(ticks, velocity);
		right_mg.move_absolute(ticks, velocity);

		while (fabs(left_mg.get_position() - ticks) > 10 || fabs(right_mg.get_position() - ticks) > 10) {
			pros::delay(10);
		}
		left_mg.brake();
		right_mg.brake();
	}

	// Turn a given number of degrees (positive = right, negative = left)
	void turnDegrees(double degrees, int velocity = 100) {
		inertial_sensor.tare_rotation(); //! Not sure if we are using rotation or heading
		double target = degrees;
		int direction = (degrees > 0) ? 1 : -1;

		left_mg.move(direction * velocity);
		right_mg.move(-direction * velocity);

		while (fabs(inertial_sensor.get_rotation() - target) > 2) {
			pros::delay(10);
		}
		left_mg.brake();
		right_mg.brake();
	}

	// Opcontrol loop method
	// The actual loop is opcontrol(), this is just called within it
	void opcontrolLoop(pros::Controller& controller) {
		int left = 0, right = 0;
		switch (opcontrol_mode) {
			case OpControlMode::LEFT_ARCADE: {
				int dir = controller.get_analog(ANALOG_LEFT_Y);
				int turn = controller.get_analog(ANALOG_LEFT_X);
				left = dir - turn;
				right = dir + turn;
				break;
			}
			case OpControlMode::RIGHT_ARCADE: {
				int dir = controller.get_analog(ANALOG_RIGHT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);
				left = dir - turn;
				right = dir + turn;
				break;
			}
			case OpControlMode::SPLIT_ARCADE: {
				int dir = controller.get_analog(ANALOG_LEFT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);
				left = dir - turn;
				right = dir + turn;
				break;
			}
			case OpControlMode::TANK: {
				left = controller.get_analog(ANALOG_LEFT_Y);
				right = controller.get_analog(ANALOG_RIGHT_Y);
				break;
			}
		}
		left_mg.move(left);
		right_mg.move(right);
	}
};

 void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {
	//* PATH PLANNING
	Drivetrain drivetrain;
	const int velocity = 30; 
	const int turn_velocity = 30;
	//* PATH PLANNING
	// Orient bot facing two blocks on side
	drivetrain.moveForward(18, velocity); // Go forward around 1.5 squares
	drivetrain.turnDegrees(turn_velocity); // Turn towards loader
	drivetrain.moveForward(1); // Move towards loader
	// Do intake
	// Back a little
	// Turn 180
	// Forward and score
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

// ...existing code...

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	Drivetrain drivetrain;

	while (true) {
		// Mode selection
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::LEFT_ARCADE);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::RIGHT_ARCADE);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::SPLIT_ARCADE);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::TANK);

		bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);


		if (r1) {
			drivetrain.startRunningIntake(-30);
			drivetrain.startRunningOuttake(30);
		} else if (r2){
			drivetrain.startRunningIntake(30);
			drivetrain.startRunningOuttake(-30);
		} else if (l1) {
			drivetrain.startRunningIntake(-30);
			drivetrain.stopOuttake();
		} else if (l2){
			drivetrain.startRunningIntake(30);
			drivetrain.stopOuttake();
		} else {
			drivetrain.stopIntake();
			drivetrain.stopOuttake();
		}

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		}


		pros::lcd::print(0, "Mode: %d", static_cast<int>(drivetrain.getOpcontrolMode()));



		drivetrain.opcontrolLoop(master);
		pros::delay(20);
	}
}

/*
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme

		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}
*/