#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

//MotorGroup left({13, 11});
//MotorGroup right({-18, 20});
const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;

MotorGroup left({1, 11});
MotorGroup right({-19, 20});
std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors(left, right) // left motor is 1, right motor is 2 (reversed)
        .withGains(
                {0.0008, 0.0004, 0.00005}, // distance controller gains 0.00001
                {0.0008, 0, 0.00001}, // turn controller gains
                {0.0008, 0.000001, 0.00005}  // angle controller gains (helps drive straight)0.0001
        )
        .withSensors(
                ADIEncoder{'E', 'F'}, // left encoder in ADI ports A & B
                ADIEncoder{'G', 'H'},  // right encoder in ADI ports C & D (reversed)
                ADIEncoder{'C', 'D'}  // middle encoder in ADI ports E & F
        )
                // green gearset, tracking wheel diameter (2.75 in), track (7 in), and TPR (360)
                // 1 inch middle encoder distance, and 2.75 inch middle wheel diameter
        .withDimensions(AbstractMotor::gearset::green, {{2.75_in, 9.5_in, 4_in, 2.75_in}, quadEncoderTPR})
        .withOdometry() // use the same scales as the chassis (above)
        .buildOdometry(); // build an odometry chassis

std::shared_ptr<ChassisController> drive =
        ChassisControllerBuilder()
                .withMotors(left, right)
                .withDimensions(AbstractMotor::gearset::green, {{4_in, 14.5_in}, imev5GreenTPR})
                .build();

std::shared_ptr<AsyncMotionProfileController> profileControllerf =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
    0.5, // Maximum linear velocity of the Chassis in m/s
    1, // Maximum linear acceleration of the Chassis in m/s/s
    5 // Maximum linear jerk of the Chassis in m/s/s/s
        })
        .withOutput(*chassis)
        .buildMotionProfileController();

std::shared_ptr<AsyncMotionProfileController> profileController =
    AsyncMotionProfileControllerBuilder()
        .withLimits({
    0.25, // Maximum linear velocity of the Chassis in m/s
    0.5, // Maximum linear acceleration of the Chassis in m/s/s
    2.5 // Maximum linear jerk of the Chassis in m/s/s/s
        })
        .withOutput(*chassis)
        .buildMotionProfileController();

std::shared_ptr<AsyncPositionController<double, double>> jawcontroller =
    AsyncPosControllerBuilder()
        .withMotor(12) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
        .build();

std::shared_ptr<AsyncPositionController<double, double>> liftcontroller =
    AsyncPosControllerBuilder()
        .withMotor(17) // lift motor port 3
//        .withGains({liftkP, liftkI, liftkD})
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
    tip.setBrakeMode(AbstractMotor::brakeMode::hold);
    arm.setGearing(AbstractMotor::gearset::red);
    jaw.setGearing(AbstractMotor::gearset::red);
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
//    std::string h = move(360, 75);
    double gearing = (double)left.getGearing();
    profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "A");
    profileController->setTarget("A");
    liftcontroller->setTarget(2000);
    jawcontroller->setTarget(500);
    profileController->waitUntilSettled();
    liftcontroller->waitUntilSettled();
    jawcontroller->waitUntilSettled();
    liftcontroller->tarePosition();
    profileController->setTarget("A", true);
    liftcontroller->setTarget(-500);
    jawcontroller->setTarget(-200);
    profileController->waitUntilSettled();
    pros::lcd::set_text(5, "hl");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "A");
//    profileController->setTarget("A");
//    jawcontroller->setTarget(1000);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, -32_in, 0_deg}}, "B");
//    profileController->waitUntilSettled();
//
//    jawcontroller->setTarget(-1000);
//    pros::delay(250);
//    profileController->setTarget("B");
//    liftcontroller->setTarget(2750);
//    profileController->removePath("A");
//    profileController->waitUntilSettled();
//    profileController->removePath("B");
//
//    liftcontroller->setTarget(-100);
//    pros::delay(250);
//    jawcontroller->setTarget(1000);
//    pros::delay(250);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 40_in, 45_deg}}, "C");
//    profileController->setTarget("C", true);

//    profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 26_in, 0_deg}}, "A");
//    profileController->setTarget("A");
//    jawcontroller->setTarget(1000);
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {34_in, 0_in, 0_deg}}, "B");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("B");
//    jawcontroller->setTarget(-1000);
//    pros::delay(1000);
//    liftcontroller->setTarget(2750);
//    profileController->removePath("A");
//    profileController->waitUntilSettled();
//    profileController->removePath("B");
//
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "C");
//    liftcontroller->setTarget(-400);
//    jawcontroller->setTarget(1000);
//    profileController->setTarget("C", true);
//    liftcontroller->setTarget(-1900);


//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "A");
//    profileController->setTarget("A");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {32_in, 28_in, 0_deg}}, "B");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("B", true); // true means to use the path in reverse
//    profileController->removePath("A");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {36_in, 0_in, 0_deg}}, "C");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("C");
//    profileController->removePath("B");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {24_in, 0_in, 0_deg}}, "D");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("D",true); // true means to use the path in reverse
//    profileController->removePath("C");
//    profileController->generatePath({{0_in, 0_in, 0_deg}, {12_in, 0_in, 0_deg}}, "E");
//    profileController->waitUntilSettled();
//
//    chassis->setMaxVelocity(gearing*.25); // set the turn to be slower for more accuracy
//    chassis->turnAngle(-270_deg);
//    chassis->setMaxVelocity(gearing); // set veolcity back to full cartride value
//
//    profileController->setTarget("E");
//    profileController->waitUntilSettled();
//    profileController->removePath("E");

//    profileController->generatePath({{0_in, 0_in, 0_deg}, {24_in, 0_in, 0_deg}}, "C");
//    profileController->waitUntilSettled();
//
//    profileController->setTarget("C");
//    profileController->removePath("B");

//    chassis->setState({0_in, 0_in, 0_deg});
////    chassis->setMaxVelocity(150);
////    chassis->moveDistance(24_in);
//    chassis->driveToPoint({0_ft, 2_ft});
////    chassis->waitUntilSettled();
//    chassis->turnAngle(45_deg);
//    chassis->moveDistance(4_in);
//    chassis->turnAngle(-45_deg);
//    chassis->moveDistance(1_ft);
//    chassis->

//    chassis->driveToPoint({1_ft, 1_ft});
//    chassis->turnAngle(-45_deg);
//    chassis->stop();

//    drive->stop();

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
        double *pos = position();
        pros::lcd::set_text(0, std::to_string(pos[0]));
        pros::lcd::set_text(1, std::to_string(pos[1]));
        pros::lcd::set_text(2, std::to_string(pos[2]));
        drive->getModel()->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
                                  controller.getAnalog(okapi::ControllerAnalog::leftX));
        ControllerButton armUpButton(ControllerDigital::R1);
        ControllerButton armDownButton(ControllerDigital::R2);
        ControllerButton clawUp(ControllerDigital::L1);
        ControllerButton clawDown(ControllerDigital::L2);
        ControllerButton tipup(ControllerDigital::A);
        ControllerButton tipdown(ControllerDigital::B);
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
        if (tipup.isPressed()){
            tip.moveVoltage(12000);
        } else if (tipdown.isPressed()){
            tip.moveVoltage(-12000);
        } else{
            tip.moveVoltage(0);
        }
//        if (runAutoButton.changedToPressed()) {
//            drive->moveDistance(6_in); // Drive forward 12 inches
//            drive->stop();
////            drive->turnAngle(90_deg);   // Turn in place 90 degrees
////            drive->stop();
//            // Drive the robot in a square pattern using closed-loop control
////            for (int i = 0; i < 4; i++) {
////                drive->moveDistance(6_in); // Drive forward 12 inches
////                drive->stop();
////                drive->turnAngle(90_deg);   // Turn in place 90 degrees
////                drive->stop();
////            }
//        }
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
