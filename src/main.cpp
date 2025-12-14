#include "main.h"

//* ---------- GLOBAL VARIABLES ----------
const double pi = 3.1415926535;
//* ---------- GLOBAL ENUMS ----------
enum class ControlConstant {KP, KI, KD};

/*
 * Drivetrain
 * All values are initalized using default values and default constructor
 * Can (and should) be initalized as: Drivetrain drivetrain;
 * Not sure if multiple instances in the same scope is ok.
 */
class Drivetrain {
private:
	//* ---------- PROS INITALIZATION ----------
	pros::MotorGroup left_mg = {1, -4, -5};
	pros::MotorGroup right_mg = {-2, 3, 6};
	pros::Imu inertial_sensor = 7;
	pros::Motor intake = 8;
	pros::Motor outtake = 9;
	pros::adi::Pneumatics intake_arm = pros::adi::Pneumatics('A', false);
	pros::adi::Pneumatics outtake_elevator = pros::adi::Pneumatics('B', false);


	//* ---------- PRIMATIVE MEMBERS ----------
	const int forward_voltage = 50;
	const int turn_voltage = 40;
	const int intake_voltage = 120;
	const int outtake_voltage = -120;
	const double wheel_diameter = 3.25;

public:
	enum class OpControlMode { LEFT_ARCADE, RIGHT_ARCADE, SPLIT_ARCADE, TANK };
	OpControlMode opcontrol_mode = OpControlMode::SPLIT_ARCADE;

	/*
	* Default constructor
	* All members are set via default values, only configuration goes here
	* Using setters here rather then configuring in constructors will hopefully combat PROS' constantly changing api
	*/
	Drivetrain(){
		left_mg.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
		right_mg.set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_DEGREES);
	}

	/*
	* Moves the bot up if it is down and vice versa
	* Returns true if successful and false if failed
	*/
	bool toggleBotHeight() {
		const char success = 1;
		bool extended = outtake_elevator.is_extended();
		if (extended)
			return outtake_elevator.retract() == success;
		else
			return outtake_elevator.extend() == success;
	}

	/*
	* Moves the intake arm up if is down and vice versa()
	* Returns true if successful and false if failed
	*/
	bool moveIntakeArm() {
		const char success = 1;
		bool extended = intake_arm.is_extended();
		if (extended)
			return intake_arm.retract() == success;
		else
			return intake_arm.extend() == success;
	}
	


	/*
	* Runs intake at default voltage, with a optional boolean parameter to reverse the direction
	*/
	void startRunningIntake(bool reversed = false) {
		if (reversed)
			intake.move(-intake_voltage);
		else
			intake.move(intake_voltage);
	}
	/*
	* Voltage overload, sets intake voltage, should be from -127 to 127 inclusive
	* If voltage is out of bounds it will be set to the closest value.
	* Set negative voltage to reverse
	*/
	void startRunningIntake(int voltage) {
		int voltage_low_bound = -127;
		int voltage_high_bound = 127;
		int valid_voltage = std::clamp(voltage, voltage_low_bound, voltage_high_bound);
		intake.move(valid_voltage);
	}

	
	/*
	* Runs outtake at default voltage, with a optional boolean parameter to reverse the direction
	*/
	void startRunningOuttake(bool reversed = false) {
		if (reversed)
			outtake.move(-outtake_voltage);
		else
			outtake.move(outtake_voltage);
	}
	/*
	* Voltage overload, sets outtake voltage, should be from -127 to 127 inclusive
	* If voltage is out of bounds it will be set to the closest value.
	* Set negative voltage to reverse
	*/
	void startRunningOuttake(int voltage) {
		int voltage_low_bound = -127;
		int voltage_high_bound = 127;
		int valid_voltage = std::clamp(voltage, voltage_low_bound, voltage_high_bound);
		intake.move(valid_voltage);
	}

	/*
	* Stops the intake if it's moving
	*/
	void stopIntake() {
		intake.brake();
	}

	/*
	* Stops the outtake if it's moving
	*/
	void stopOuttake() {
		outtake.brake();
	}

	/*
	* Setter for opcontrol_mode
	*/
	void setOpcontrolMode(OpControlMode mode) {
		opcontrol_mode = mode;
	}

	/*
	* Getter for opcontrol_mode
	*/
	OpControlMode getOpcontrolMode() const {
		return opcontrol_mode;
	}

	/*
	* Move forward a number of inches using default velocity
	*/
	void move(double inches){ 
		double revs = inches / (pi * wheel_diameter);
		int degrees = static_cast<int>(revs * 360);
		left_mg.move_relative(degrees, forward_voltage); //* technicly takes in RPM not voltage but it works anyway
		right_mg.move_relative(degrees, forward_voltage);
	}
	/*
	* Move forward a number of inches using a set velocity
	* Use negative voltage to go backwards
	*/
	void move(double inches, int rpm){
		double revs = inches / (pi * wheel_diameter);
		int degrees = static_cast<int>(revs * 360);
		left_mg.move_relative(degrees, rpm);
		right_mg.move_relative(degrees, rpm);
	}

	/*
	* Turns using PID
	* target_in_degrees: the target rotation to turn to, positive is clockwise
	* kP: proportional constant, gets smaller as target gets closer 
	* kI: integral constant, use to fix steady state error
	* kD: derivative constant, used to prevent overshooting
	* All constants should be between 0 and 1. kP is usually much larger than kI and kD.

	*/
	void turn(double target_in_degrees, double kP = 0.1, double kI = 0.0, double kD = 0.0) {
		kP = std::clamp(kP, 0.0, 1.0);
		kI = std::clamp(kP, 0.0, 1.0);
		kD = std::clamp(kP, 0.0, 1.0);
		double error = target_in_degrees;
		double previous_error = error;
		double integral = 0.0;
		double derivative = 0.0;
		double rotation = 0.0;
		int left_voltage = 0;
		int right_voltage = 0;

		if (inertial_sensor.tare_rotation() == PROS_ERR){
			while (1){
				pros::lcd::print(0 , "Failed to zero rotation, errno = %d", errno);
				pros::delay(2);
			}
		}
		pros::delay(2000);

		pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
		while (1) {
			
			if (fabs(error) < 3.0){
				pros::lcd::print(1, "err: %d", (int)error);
				pros::lcd::print(2, "prev: %d", (int)previous_error);
				pros::lcd::print(3, "target: %d", (int)target_in_degrees);
				pros::lcd::print(4, "lvolt: %d", (int)left_voltage);
				pros::lcd::print(5, "rvolt: %d", (int)right_voltage);
				pros::lcd::print(6, "ended");
				break;
			}

			rotation = inertial_sensor.get_rotation();
			if (rotation == PROS_ERR_F){
				pros::lcd::print(6, "IMU broke");
				if (errno == ENXIO)
					pros::lcd::print(7, "wrong IMU port");
				else if (errno == ENODEV)
					pros::lcd::print(7, "'port cannot be configured as an inertal sensor', whatever that means");
				else if (errno == EAGAIN)
					pros::lcd::print(7, "IMU Calibrating");
			}

			previous_error = error;
			error = target_in_degrees - rotation;
			integral += error;
			derivative = error - previous_error;
			left_voltage = (int)(kP * error + kI * integral + kD * derivative);
			right_voltage = -(int)(kP * error + kI * integral + kD * derivative);

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);
			pros::lcd::print(1, "rotation: %d err: %d", (int)rotation, (int)error);
			pros::lcd::print(2, "prev: %d", (int)previous_error);
			pros::lcd::print(3, "target: %d", (int)target_in_degrees);
			pros::lcd::print(4, "lvolt: %d", (int)left_voltage);
			pros::lcd::print(5, "rvolt: %d", (int)right_voltage);
			pros::delay(2);	
		}
		left_mg.brake();
		right_mg.brake();
	}

	// Opcontrol loop method
	// The actual loop is opcontrol(), this is just called within it
	void opcontrolMove(pros::Controller& controller) {
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
	Drivetrain drivetrain;
	drivetrain.move(12);
	pros::delay(1000);
	drivetrain.turn(-90, 0.8, 0.02);
	drivetrain.move(41);
	pros::delay(1000);
	drivetrain.turn(-90, 0.8, 0.02);
	pros::lcd::print(0, "secs: %d", pros::millis() / 1000);
	drivetrain.moveIntakeArm();
	drivetrain.startRunningIntake(true);
	drivetrain.move(25);
	pros::delay(1500);
	drivetrain.move(-1);
	pros::delay(500);
	drivetrain.move(1);
	pros::delay(500);
	drivetrain.stopIntake();
	drivetrain.move(-15); // tile - despenser - half bot length
	drivetrain.moveIntakeArm();
	//drivetrain.turn(170, 0.8, 0.04, 0.5);
	drivetrain.turn(-90, 0.8, 0.02);
	drivetrain.turn(-90, 0.8, 0.02);
	drivetrain.toggleBotHeight();
	drivetrain.move(15);
	drivetrain.startRunningOuttake(true);
	drivetrain.startRunningIntake(true);
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
			drivetrain.toggleBotHeight();
		}

		if (b) {
			drivetrain.moveIntakeArm();
		}

		drivetrain.opcontrolMove(master);
		pros::delay(20);
	}
}
