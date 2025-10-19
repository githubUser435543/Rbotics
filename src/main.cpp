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
	pros::adi::Pneumatics intakeArm = pros::adi::Pneumatics('A', false);
	pros::adi::Pneumatics outtakeArm = pros::adi::Pneumatics('B', false);

	const int forwardVolatage = 30;
	const int turnVoltage = 30;
	const int intake_voltage = 120;
	const int outtake_voltage = -120;
	const double wheel_diameter = 3.25;
	const double ticks_per_rev = 600.0;
	bool intake_state = 0;
	bool outtake_state = 0;

public:
	enum class OpControlMode { LEFT_ARCADE, RIGHT_ARCADE, SPLIT_ARCADE, TANK };
	OpControlMode opcontrol_mode = OpControlMode::SPLIT_ARCADE;

	void moveMotors() {
		left_mg.move(forwardVolatage);
		right_mg.move(forwardVolatage);
		pros::delay(500);
		left_mg.brake();
		right_mg.brake();
	}

	void moveOutakeArm() {
		bool extended = outtakeArm.is_extended();
		if (extended)
			outtakeArm.retract();
		else
			outtakeArm.extend();
	}

	void moveIntakeArm() {
		bool extended = intakeArm.is_extended();
		if (extended)
			intakeArm.retract();
		else
			intakeArm.extend();
	}
	

	void startRunningIntake() {
		intake.move(intake_voltage);
	}
	void startRunningIntake(int voltage) {
		intake.move(voltage);
	}
	void startRunningIntake(bool reversed) {
		if (reversed)
			intake.move(-intake_voltage);
		else
			intake.move(intake_voltage);
	}

	void startRunningOuttake() {
		outtake.move(outtake_voltage);
	}
	void startRunningOuttake(int voltage) {
		outtake.move(voltage);
	}
	void startRunningOuttake(bool reversed) {
		if (reversed)
			outtake.move(-outtake_voltage);
		else
			outtake.move(outtake_voltage);
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
	void moveForward(double inches) {
		double revs = inches / (pi * wheel_diameter);
		int ticks = static_cast<int>(revs * ticks_per_rev);

		left_mg.tare_position();
		right_mg.tare_position();

		left_mg.move_absolute(ticks, forwardVolatage);
		right_mg.move_absolute(ticks, forwardVolatage);

		left_mg.brake();
		right_mg.brake();
	}

	// Turn a given number of degrees (positive = right, negative = left)
	void turnDegrees(double target_in_degrees, double kP = 0.1, double kI = 0.0, double kD = 0.0) {
		inertial_sensor.tare_rotation();

		double error = target_in_degrees - inertial_sensor.get_rotation();
			

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
				int turn = 0.5 * controller.get_analog(ANALOG_LEFT_X);
				left = dir + turn;
				right = dir - turn;
				break;
			}
			case OpControlMode::RIGHT_ARCADE: {
				int dir = controller.get_analog(ANALOG_RIGHT_Y);
				int turn = controller.get_analog(ANALOG_RIGHT_X);
				left = dir + turn;
				right = dir - turn;
				break;
			}
			case OpControlMode::SPLIT_ARCADE: {
				int dir = controller.get_analog(ANALOG_LEFT_Y);
				int turn = 0.5 * controller.get_analog(ANALOG_RIGHT_X);
				left = dir + turn;
				right = dir - turn;
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
	// Orient bot facing two blocks on side
	
	 // Go forward around 1.5 squares

	 // Turn towards loader
	 // Move towards loader
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
		/*
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::LEFT_ARCADE);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::RIGHT_ARCADE);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::SPLIT_ARCADE);
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
			drivetrain.setOpcontrolMode(Drivetrain::OpControlMode::TANK);
		*/
		bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		bool x = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
		bool b = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);


		if (r1) {
			drivetrain.startRunningIntake(true);
			drivetrain.startRunningOuttake(true);
		} else if (r2){
			drivetrain.startRunningIntake();
			drivetrain.startRunningOuttake();
		} else if (l1) {
			drivetrain.startRunningIntake(true);
			drivetrain.stopOuttake();
		} else if (l2){
			drivetrain.startRunningIntake();
			drivetrain.stopOuttake();
		} else {
			drivetrain.stopIntake();
			drivetrain.stopOuttake();
		}

		if (x) {
			drivetrain.moveOutakeArm();
		}

		if (b) {
			drivetrain.moveIntakeArm();
		}


		pros::lcd::print(0 /*line*/, "Mode: %d", static_cast<int>(drivetrain.getOpcontrolMode()));

		drivetrain.opcontrolLoop(master);
		pros::delay(20);
	}
}