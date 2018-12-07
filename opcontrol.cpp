#include "C:\Users\RD SINGH\Desktop\General\Robotics\ShawniganLakeCode\include\main.h"
#include <sstream>
/*
#ifndef max
#define max (a,b) (((a) > (b)) ? a : b)
#endif

#ifndef min
#define min (a, b) (((a) < (b)) ? a : b)
#endif
*/
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

 void move(double targetDistance, int maxPower)
{
	double kp = 0.75;
	double kd = 0.02;
  double MIN_POWER = 1;
	/* Variables used to calculate how fast bot is moving*/
	double wheelDiameter = 4;
	double circumference = wheelDiameter * M_PI; //Roughly 14.13 inches
	double currentDistance = 0;
	double error = 1, lastError = 0, integral = 0;
	double derivative = 0;
	int power = 0, powerLeft = 0, powerRight = 0;

  pros::Motor lfdMotor (1, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rfdMotor (2, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor lbdMotor (3, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rbdMotor (4, pros::E_MOTOR_ENCODER_DEGREES);


	while(error != 0)
	{
		currentDistance = (lfdMotor.get_position() + rfdMotor.get_position() + rbdMotor.get_position() + lbdMotor.get_position())/4;

		error = targetDistance - currentDistance;


		derivative = error - lastError;
		lastError = error;

		power = (error * kp) + (derivative * kd);

		if(power > 0)
		{
			if(power > maxPower)
			{
				power = maxPower;
			}
			else if(power < MIN_POWER)
			{
				power = MIN_POWER;
			}
		}
		else if(power < 0)
		{
			if(power < -maxPower)
			{
				power = -maxPower;
			}
			else if(power > -MIN_POWER)
			{
				power = -MIN_POWER;
			}
		}
		powerLeft = power;
		powerRight = power;

    lfdMotor.move(powerLeft);
    rfdMotor.move(-powerRight);
    lbdMotor.move(powerLeft);
    rbdMotor.move(-powerRight);


	}

}

void visionSense()
{

  pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

  pros::Vision vision_sensor (9);

  pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, 1);

  double topHeight = obj.top_coord;

  double pixHeight = obj.height;
  double pixWidth = obj.width;

  double pToIRatioW = pixWidth/5.12;
  double pToIRatioL = pixHeight/5.31;

  double ratio = (pToIRatioL + pToIRatioW)/2;

  double height = (topHeight/ratio);

  double verticalIntitial = sqrt(height * 771.65);

  double speed = (verticalIntitial/tan(34));

  double rpm = (((speed * 60)/12.566)/21);


  flyLWheel_mtr.move(rpm);
  flyRWheel_mtr.move(rpm);

  pros::delay(50);

}

void driveTurn1(int speed)
{
  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

  leftBDrive_mtr.move(speed);
  rightBDrive_mtr.move(-speed);
  rightFDrive_mtr.move(-speed);
  leftFDrive_mtr.move(speed);
}

int auton = 0;



void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

	pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor chainBar_mtr(8, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::ADIPort limitSwitch('A');

	while (true) {

    pros::lcd::print(0, std::string(std::to_string(leftBDrive_mtr.get_position())).c_str());

		/*pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);*/
		leftBDrive_mtr.move(((master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_LEFT_X)/2)*1.5));
		leftFDrive_mtr.move(((master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_LEFT_X)/2)*1.5));
		rightFDrive_mtr.move(((master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_LEFT_X)/2)*1.5));
		rightBDrive_mtr.move(((master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_LEFT_X)/2)*1.5));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
      //visionSense();
      flyLWheel_mtr.move(-95);
      flyRWheel_mtr.move(-95);
		}
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
      flyLWheel_mtr.move(-91);
      flyRWheel_mtr.move(-91);
    }
		else
		{
			flyLWheel_mtr.move(0);
			flyRWheel_mtr.move(0);
		}

    chainBar_mtr.move(master.get_analog(ANALOG_RIGHT_Y));

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			ballIntake_mtr.move(100);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			ballIntake_mtr.move(-87);
		}
		else
		{
			ballIntake_mtr.move(0);
		}

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
      chainBar_mtr.move(127);

      pros::delay(80);

      chainBar_mtr.move(0);
    }

    pros::delay(20);
	}
}
