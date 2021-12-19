#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

//MotorGroup left({13, 11});
//MotorGroup right({-18, 20});

MotorGroup left({1, 11});
MotorGroup right({10, 20});
std::shared_ptr<OdomChassisController> chassis =
        ChassisControllerBuilder()
                .withMotors(left, right) // left motor is 1, right motor is 2 (reversed)
//                .withGains(
//                        {0.001, 0, 0.0001}, // distance controller gains
//                        {0.001, 0, 0.0001}, // turn controller gains
//                        {0.001, 0, 0.0001}  // angle controller gains (helps drive straight)
//                )
//                .withSensors(
//                        ADIEncoder{'C', 'D'}, // left encoder in ADI ports A & B
//                        ADIEncoder{'E', 'F'},  // right encoder in ADI ports C & D (reversed)
//                        ADIEncoder{'G', 'H'}  // middle encoder in ADI ports E & F
//                )
                        // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
                        // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
                .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
                .withOdometry() // use the same scales as the chassis (above)
                .buildOdometry(); // build an odometry chassis

std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()
                .withMotors(left, right)
                .withDimensions(AbstractMotor::gearset::green, {{4_in, 14.5_in}, imev5GreenTPR})
                .build();

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
    arm.setBrakeMode(AbstractMotor::brakeMode::hold);
    jaw.setBrakeMode(AbstractMotor::brakeMode::hold);
    autonomous();

//    lv_obj_align(myLabel, NULL, LV_ALIGN_LEFT_MID, 10, 0); //set the position to center
//    autonomous();

//	DLF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//	DLB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//	DRF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//	DRB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
/////    pros::Imu imu_sensor(6);
//    imu_sensor.reset();
//	autonomous();
	// opcontrol();
	// pros::lcd::register_btn1_cb(on_center_button);
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
    drive->moveDistance(6_in);
//    chassis->setState({0_in, 0_in, 0_deg});
//    chassis->turnAngle(45_deg);
//    chassis->moveDistance(1.4_ft);
//    Sensors_reset();
//    std::string moveVal = move(416, 90);
//    pros::lcd::set_text(0, moveVal);
////    std::string moveVal2 = move(5, 127);
////    pros::lcd::set_text(0, moveVal2);
//    pros::lcd::set_text(0, "hello");
//    pros::delay(50);
//    std::string turnVal = turn(1.5, 50);
//    pros::lcd::set_text(0, turnVal);
//    double *pos = position();
//    pros::lcd::set_text(1, std::to_string(pos[0]));
//    pros::lcd::set_text(2, std::to_string(pos[1]));
//    pros::lcd::set_text(3, std::to_string(pos[2]));

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
    while(true){
        drive->getModel()->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                                  controller.getAnalog(okapi::ControllerAnalog::leftX));
        ControllerButton armUpButton(ControllerDigital::R1);
        ControllerButton armDownButton(ControllerDigital::R2);
        ControllerButton clawUp(ControllerDigital::L1);
        ControllerButton clawDown(ControllerDigital::L2);
        ControllerButton runAutoButton(ControllerDigital::X);
        if (armUpButton.isPressed()) {
            arm.moveVoltage(12000);
        } else if (armDownButton.isPressed() && arm.getPosition() != 0) {
            arm.moveVoltage(-12000);
        } else {
            arm.moveVoltage(0);
        }
        if (clawUp.isPressed()){
            jaw.moveVoltage(-12000);
        } else if (clawDown.isPressed() && arm.getPosition() != 0){
            jaw.moveVoltage(12000);
        } else{
            jaw.moveVoltage(0);
        }
        if (runAutoButton.changedToPressed()) {
            drive->moveDistance(6_in); // Drive forward 12 inches
            drive->stop();
//            drive->turnAngle(90_deg);   // Turn in place 90 degrees
//            drive->stop();
            // Drive the robot in a square pattern using closed-loop control
//            for (int i = 0; i < 4; i++) {
//                drive->moveDistance(6_in); // Drive forward 12 inches
//                drive->stop();
//                drive->turnAngle(90_deg);   // Turn in place 90 degrees
//                drive->stop();
//            }
        }
//        pros::lcd::set_text(0, std::to_string(encoder_left.get_value()));
//        pros::lcd::set_text(1, std::to_string(encoder_right.get_value()));

        pros::delay(10);
    }
//    while(true) {
//        drivemotors();
//        liftArm();
//        pros::delay(10);
//    }

//	pros::lcd::set_text(2, "Hello 2 PROS!");
//    while (true){
//        drivemotors();
//    }
//

//    double *globalPos;
//    lv_obj_t *obj = drawRectangle( 25, 20, 100, 100, LV_COLOR_YELLOW);
//    lv_canvas_draw_rect()
//    Sensors_reset();

//    while (true) {
//        drive.arcade(con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
//        int xjoy = con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
//        int yjoy = con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//        pros::lcd::set_text(1, std::to_string(xjoy));
//        pros::lcd::set_text(2, std::to_string(yjoy));
//
//        pros::lcd::set_text(3, std::to_string(yjoy + xjoy));
//        pros::lcd::set_text(4, std::to_string(yjoy - xjoy));


//        drivemotors();


//        globalPos = position();
//
//        pros::lcd::set_text(3, std::to_string(globalPos[0]));
//        pros::lcd::set_text(4, std::to_string(globalPos[1]));
//        pros::lcd::set_text(5, std::to_string(globalPos[2]));

//        pros::delay(10);
//    }
//    sensor.reset();
//    while (true) {
//        for(int i = 0; i < 10000; i++){
//            pros::lcd::set_text(3, std::to_string(i));
//        }
////        pros::lcd::set_text(4, std::to_string(sensor.get_value()));
//////        std::cout << "Encoder Value: " << enco.get_value();
////        pros::delay(10);
//    }
//    imu_sensor.reset();
//    pros::lcd::set_text(3, "hello");
//	double angle = 50;
//	double dam = imu_sensor.get_rotation();
//	pros::lcd::set_text(5, std::to_string(dam));
//	if(angle < dam){
//	    pros::lcd::set_text(5, "wo");
//	}else{
//	    pros::lcd::set_text(5, "no");
//	}

}
