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
	pros::adi::Pneumatics descoreArm = pros::adi::Pneumatics('C', false);


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
		
		left_mg.set_gearing_all(pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);
		right_mg.set_gearing_all(pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN);

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
	* Moves descore arm, pretty easy to understand
	* Returns true if successful and false if failed
	*/
	bool moveDescoreArm() {
		const char success = 1;
		bool extended = descoreArm.is_extended();
		if (extended)
			return descoreArm.retract() == success;
		else
			return descoreArm.extend() == success;
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
	* Last minute hack
	*/
	void moveForTime(double milis){
		//double inital_rotation = inertial_sensor.get_rotation();
		left_mg.move(forward_voltage);
		right_mg.move(forward_voltage);
		if (milis < 0){
			milis = fabs(milis);
			left_mg.move(-forward_voltage);
			right_mg.move(-forward_voltage);
		}
		pros::delay(milis);
		left_mg.move(0);
		right_mg.move(0);
	}

	void drivePID(double inches, double kP = 0.1, double kPturn = 0.0, double kI = 0, double kD = 0) {
		kP = std::clamp(kP, 0.0, 0.5);
		kI = std::clamp(kI, 0.0, 0.1);
		kD = std::clamp(kD, 0.0, 0.1);

		const double target_degrees = (inches / (pi * wheel_diameter)) * 360.0;

		if (!left_mg.tare_position()){
			if (errno == EACCES)
				pros::lcd::print(1, "mutex stuff");
			else if (errno == ENODEV){
				pros::lcd::print(1, "port cannot be configured as motor");
			}
			return;
		}
		if (!right_mg.tare_position()){
			if (errno == EACCES)
				pros::lcd::print(1, "mutex stuff");
			return;
		}
		if (inertial_sensor.tare_rotation() == PROS_ERR){
			if (errno == EAGAIN){
				pros::lcd::print(0 , "Inertial sensor calibrating, did you wait for calibration?");
				return;
			}
			else if (errno == ENODEV){
				pros::lcd::print(0, "Port cannot be inertial sensor");
				return;
			}
			else if (errno == EAGAIN){
				pros::lcd::print(0, "Port number not 1-21");
				return;
			}
			pros::delay(10000);
		}

		double error = target_degrees;
		double prev_error = error;
		double integral = 0.0;
		bool increased_int = false;

		double left_position, right_position, rotation, average_position;
		while (true) {
			left_position = left_mg.get_position() * 2;
			right_position = right_mg.get_position() * 2;
			rotation = inertial_sensor.get_rotation();
			average_position = (left_position + right_position) * 0.5;
			pros::lcd::print(1, "L pos: %d", (int)left_position);
			pros::lcd::print(2, "R pos: %d", (int)right_position);
			pros::lcd::print(3, "Target: %d", (int)target_degrees);
			
			error = target_degrees - average_position;
			if (std::abs(error) <= 1.0)
				break;

			if (!increased_int && error < 15){
				kI *= 3;
				increased_int = true;
			}
			integral += error;
			double derivative = error - prev_error;
			prev_error = error;

			double voltage = kP * error + kI * integral + kD * derivative;
			double left_voltage = voltage + (kPturn * -rotation);
			double right_voltage = voltage + (kPturn * rotation);
			pros::lcd::print(4, "rot: %d", (int)rotation);
			pros::lcd::print(5, "lv: %d", (int)left_voltage);
			pros::lcd::print(6, "rv: %d", (int)right_voltage);

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);

			pros::delay(50);
		}

		left_mg.brake();
		right_mg.brake();
	}



	/*
	* Move forward a number of inches using default velocity
	*/
	void move(double inches){ 
		double revs = inches / (pi * wheel_diameter);
		int degrees = static_cast<int>(revs * 360);
		left_mg.move_relative(degrees, forward_voltage); //* technicly takes in RPM not voltage but it works anyway
		right_mg.move_relative(degrees, forward_voltage);
		while (left_mg.get_voltage() && right_mg.get_voltage()){
			pros::delay(2);
		}
		left_mg.move(0);
		right_mg.move(0);
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

	//* Untested
	void turnToPoint(double target, unsigned char voltage){
		inertial_sensor.tare_rotation();

		int left_voltage = voltage;
		int right_voltage = voltage;
		if (target < 0)
			left_voltage *= -1;
		else 
			right_voltage *= -1;

		while (fabs(inertial_sensor.get_rotation()) < target){
			if (target < 0){
				left_mg.move(left_voltage);
				right_mg.move(right_voltage);
			}
		}
	}

	//* Untested
	void moveToPoint(double target, unsigned char voltage, double turn_KP = 4){
		inertial_sensor.tare_rotation();
		const double target_degrees = (target / (pi * wheel_diameter)) * 360.0;
		while (true) {
			double left_position = left_mg.get_position() * 2;
			double right_position = right_mg.get_position() * 2;
			double rotation = inertial_sensor.get_rotation();
			double average_position = (left_position + right_position) * 0.5;
			pros::lcd::print(1, "L pos: %d", (int)left_position);
			//pros::lcd::print(2, "R pos: %d", (int)right_position);
			pros::lcd::print(3, "Target: %d", (int)target_degrees);
			
			if (target_degrees < average_position)
				break;

			double error = target_degrees - average_position;
			double left_voltage = voltage + (turn_KP * -rotation);
			double right_voltage = voltage + (turn_KP * rotation);
			//pros::lcd::print(4, "rot: %d", (int)rotation);
			pros::lcd::print(5, "lv: %d", (int)left_voltage);
			pros::lcd::print(6, "rv: %d", (int)right_voltage);

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);

			pros::delay(40);
		}
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
		kI = std::clamp(kI, 0.0, 1.0);
		kD = std::clamp(kD, 0.0, 1.0);
		double error = target_in_degrees;
		double previous_error = error;
		double integral = 0.0;
		double derivative = 0.0;
		double rotation = 0.0;
		long long left_voltage = 0;
		long long right_voltage = 0;

		if (inertial_sensor.tare_rotation() == PROS_ERR){
			if (errno == EAGAIN){
				pros::lcd::print(0 , "Inertial sensor calibrating, did you wait for calibration?");
				return;
			}
			else if (errno == ENODEV){
				pros::lcd::print(0, "Port cannot be inertial sensor");
				return;
			}
			else if (errno == EAGAIN){
				pros::lcd::print(0, "Port number not 1-21");
				return;
			}
		}
		pros::delay(500);

		pros::Controller master = pros::Controller(pros::E_CONTROLLER_MASTER);
		while (1) {
			
			if (abs((int)error) <= 1){
				pros::lcd::print(1, "err: %d", (int)error);
				pros::lcd::print(2, "prev: %d", (int)previous_error);
				pros::lcd::print(6, "ended");
				left_mg.move(0);
				right_mg.move(0);
				break;
			}

			rotation = inertial_sensor.get_rotation();
			if (rotation == PROS_ERR_F){
				pros::lcd::print(0, "IMU broke");
				if (errno == ENXIO)
					pros::lcd::print(1, "wrong IMU port");
				else if (errno == ENODEV)
					pros::lcd::print(1, "'port cannot be configured as an inertal sensor', whatever that means");
				else if (errno == EAGAIN)
					pros::lcd::print(1, "IMU Calibrating");
				break;
			}

			previous_error = error;
			error = target_in_degrees - rotation;
			integral += error;
			//integral *= 0.95;
			derivative = error - previous_error;
			left_voltage = (long long)(kP * error + kI * integral + kD * derivative);
			right_voltage = -(long long)(kP * error + kI * integral + kD * derivative);

			left_mg.move(left_voltage);
			right_mg.move(right_voltage);
			pros::lcd::print(1, "rotation: %d err: %d", (int)(inertial_sensor.get_rotation()), (int)error);
			pros::lcd::print(2, "lv: %d rv: %d", (int) left_voltage,(int) right_voltage);
			pros::lcd::print(3, "kP: %d err: %d m: %d", (int) (kP * 100), (int)error, (int)(error * kP));
			pros::lcd::print(4, "kI: %d int: %d m: %d", (int) (kI * 100), (int)integral, (int)(integral * kI));
			pros::lcd::print(5, "kD: %d der: %d m: %d", (int) (kD * 100), (int)derivative, (int)(derivative * kD));
			pros::delay(50);	
		}
		left_mg.brake();
		right_mg.brake();
	}

	void waitForInertial(){
		inertial_sensor.reset();
		while (inertial_sensor.is_calibrating())
			pros::delay(2);
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
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


/*
* autonomous() is a special function that is called by the operating system, 
* sometimes you might want to run something other than the autonomous there 
* thats why this function exists.
*/
void realAuton(Drivetrain& drivetrain){
	const double right_angle_turn_KP = 0.80;
	const double right_angle_turn_KI = 0.02;
	const double forward_KP = 0.08;
	const double forward_KI = 0.0025;
	const double forward_KD = 0.002;
	const double drive_PID_turn_KP = 2;
	const bool right_auton = false;
	const double swapAuton = (right_auton) ? -1 : 1;
	drivetrain.toggleBotHeight();
	drivetrain.drivePID(8, forward_KP * 0.95, drive_PID_turn_KP, forward_KI, forward_KD);
	drivetrain.turn(-88, right_angle_turn_KP, right_angle_turn_KI); 
	drivetrain.moveForTime(1160);
	drivetrain.turn(-90, right_angle_turn_KP, right_angle_turn_KI);
	drivetrain.moveIntakeArm();
	drivetrain.startRunningIntake(true);
	drivetrain.moveForTime(880);
	pros::delay(1000);
	drivetrain.moveForTime(-500);
	drivetrain.moveForTime(700);
	pros::delay(1000);
	drivetrain.stopIntake();
	drivetrain.startRunningIntake(true);
	drivetrain.moveForTime(-700);
	drivetrain.moveIntakeArm();
	drivetrain.turn(-90, right_angle_turn_KP, right_angle_turn_KI);
	drivetrain.turn(-90, right_angle_turn_KP, right_angle_turn_KI);
	drivetrain.moveForTime(600);
	drivetrain.startRunningOuttake(true); // startRunningInstake(true)


	/*
	drivetrain.toggleBotHeight();
	drivetrain.drivePID(8, forward_KP * 0.95, drive_PID_turn_KP, forward_KI, forward_KD);
	drivetrain.turn(swapAuton * -88 , right_angle_turn_KP, right_angle_turn_KI); 
	drivetrain.drivePID(35, forward_KP, drive_PID_turn_KP, forward_KI * 0.8, 0.1);
	drivetrain.turn(swapAuton * -90, right_angle_turn_KP, right_angle_turn_KI);
	drivetrain.moveIntakeArm();
	drivetrain.startRunningIntake(true);
	drivetrain.drivePID(10, forward_KP * 2, drive_PID_turn_KP, forward_KI);
	drivetrain.drivePID(-3, forward_KP, drive_PID_turn_KP, forward_KI);
	drivetrain.drivePID(3, forward_KP, drive_PID_turn_KP, forward_KI);
	drivetrain.stopIntake();
	drivetrain.startRunningIntake(true);
	drivetrain.drivePID(-10, forward_KP, drive_PID_turn_KP, forward_KI);
	drivetrain.moveIntakeArm();
	drivetrain.turn(swapAuton * -90, right_angle_turn_KP, right_angle_turn_KI);
	drivetrain.turn(swapAuton * -90, right_angle_turn_KP, right_angle_turn_KI);
	drivetrain.moveForTime(500);
	drivetrain.startRunningOuttake(true);
	*/
}

Drivetrain drivetrain;
void autonomous() {
	//* Try using only motor.move() instead of move realtive
	drivetrain.waitForInertial();
	//drivetrain.moveForTime(300);
	
	drivetrain.moveToPoint(20, 20);

}

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	while (true) {
		bool r1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool r2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
		bool x = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X);
		bool b = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
		bool up = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);


		if (r1) {
			drivetrain.startRunningIntake(true);
			drivetrain.startRunningOuttake();
		} else if (r2){
			drivetrain.startRunningIntake();
			drivetrain.startRunningOuttake(true);
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

		if (up) {
			drivetrain.moveDescoreArm();
		}

		drivetrain.opcontrolMove(master);
		pros::delay(20);
	}
}
