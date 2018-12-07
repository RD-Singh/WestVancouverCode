#include <math.h>
#include <C:\Users\RD SINGH\Desktop\General\Robotics\ShawniganLakeCode\include\main.h>

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

 int sig = 0;

double ROBO_SPEED = 1.41;

void driveMotors1(int speed)
{
  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

  leftBDrive_mtr.move(speed);
  rightBDrive_mtr.move(speed);
  rightFDrive_mtr.move(speed);
  leftFDrive_mtr.move(speed);
}

void driveMotors(int speed)
{
  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

  leftBDrive_mtr.move(speed);
  rightBDrive_mtr.move(speed);
  rightFDrive_mtr.move(speed);
  leftFDrive_mtr.move(speed);
}

void driveTurn(int degrees, int speed)
{
  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

  degrees*=3.6;

  leftBDrive_mtr.move_relative(degrees, speed);
  rightBDrive_mtr.move_relative(degrees, speed);
  rightFDrive_mtr.move_relative(degrees, speed);
  leftFDrive_mtr.move_relative(degrees, speed);
}

void resetSensor(int target)
{


  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

  leftFDrive_mtr.set_zero_position(target);
  rightBDrive_mtr.set_zero_position(target);
  leftBDrive_mtr.set_zero_position(target);
  rightFDrive_mtr.set_zero_position(target);

}

void encoderPID(int power)
{
  double kp = 0.695;

	int main = 0;
	int secondary = 0;
	int error = 1;
  int powerLeft, powerRight;

  int maxPower = power;
  int MIN_POWER = maxPower - 20;

  pros::Motor lfdMotor (1, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rfdMotor (2, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor lbdMotor (3, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rbdMotor (4, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

  //int leftPos = (abs(lfdMotor.get_position()) + abs(lbdMotor.get_Position())/2;
	main = (abs(rfdMotor.get_position()) >= abs(lfdMotor.get_position())) ? abs(rfdMotor.get_position()) : abs(lfdMotor.get_position());
	secondary = (abs(rfdMotor.get_position()) >= abs(lfdMotor.get_position())) ? abs(lfdMotor.get_position()) : abs(rfdMotor.get_position());

	error = main - secondary;

	if(main > secondary)
	{
			power = power - error * kp;
	}

  if(power > 0)
  {
    if(power < MIN_POWER)
    {
      power = MIN_POWER;
    }
  }
  else if(power < 0)
  {
    if(power > -MIN_POWER)
    {
      power = -MIN_POWER;
    }
  }

  if(rfdMotor.get_position() > lfdMotor.get_position())
  {
    powerLeft = maxPower;
    powerRight = power;
  }
  else
  {
    powerLeft = power;
    powerRight = maxPower;
  }


  lfdMotor.move(powerLeft);
  rfdMotor.move(powerRight);
  lbdMotor.move(powerLeft);
  rbdMotor.move(powerRight);

}

void driveBrakeHold()
{
  pros::Motor lfdMotor (1, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rfdMotor (2, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor lbdMotor (3, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rbdMotor (4, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  lfdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lbdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rfdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rbdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

}

void resetBrake()
{
  pros::Motor lfdMotor (1, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rfdMotor (2, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor lbdMotor (3, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rbdMotor (4, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  lfdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  lbdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rfdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  rbdMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

}


 void moveDrive(double targetDistance, int maxPower, int flyWheelP, int flipperP, int intakeP)
{
	double kp = 0.695;

	int currentDistance = 0;
	double error = 20;

  int targ = (targetDistance/12.566) * 360;

  int leftF, leftB, rightF, rightB;

  pros::Motor lfdMotor (1, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rfdMotor (2, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor lbdMotor (3, MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor rbdMotor (4, MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

  pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor flip_mtr(8, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);


	while(error != 0)
	{
    leftF = abs(lfdMotor.get_position());
    leftB = abs(lbdMotor.get_position());
    rightF = abs(rfdMotor.get_position());
    rightB = abs(rbdMotor.get_position());

		currentDistance = (leftF + rightF)/2;

		error = targ - currentDistance;

    ballIntake_mtr.move(intakeP);

    flyLWheel_mtr.move(flyWheelP);
    flyRWheel_mtr.move(flyWheelP);

    flip_mtr.move(flipperP);

    encoderPID(maxPower);

    if(error < 15 && error > -15)
    {
      error = 0;
      driveMotors(0);
      driveBrakeHold();
    }
  }
  resetBrake();

  ballIntake_mtr.move(0);
  flip_mtr.move(0);
}


void resetPos()
{
  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
	pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
	pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

  leftBDrive_mtr.tare_position();
  rightBDrive_mtr.tare_position();
  rightFDrive_mtr.tare_position();
  leftFDrive_mtr.tare_position();
}

void targetTurn(int speed)
{
  pros::Motor leftBDrive_mtr(1, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor rightBDrive_mtr(2, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor leftFDrive_mtr(3, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor rightFDrive_mtr(4, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Vision vision_sensor(9);

  pros::vision_object_s_t obj = vision_sensor.get_by_sig(0, 1);

  int center = 320;

  int midCoord = obj.x_middle_coord;

  while(center != (midCoord + 5) || center != (midCoord -5))
  {
    leftBDrive_mtr.move(speed);
    rightBDrive_mtr.move(speed);
    leftFDrive_mtr.move(speed);
    rightFDrive_mtr.move(speed);
  }

  driveMotors(0);
  driveBrakeHold();
  resetPos();

}

void flyWheel(int speed)
{
  pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

  flyLWheel_mtr.move(speed);
  flyRWheel_mtr.move(speed);
}

void pause()
{
  pros::delay(100);
}

void skillsAuton()
{
  pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor chainBar_mtr(8, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

  resetPos();

  //Moves forward
  moveDrive(45, 77, -91, 0, 0);

  //flips cap
  moveDrive(51, 127, -91, 0, -50);

  ballIntake_mtr.move(60);

  pros::delay(575);

  //moves back
  moveDrive(0, -77, -82, 0, 40);

  resetPos();

  driveMotors1(-77);

  pros::delay(300);

  resetPos();

  pause();

  //moves a bit forward
  moveDrive(7, 57, -82, 0, 30);

  pause();

  //turns
  driveTurn(-90, -70);

  pros::delay(750);

  resetPos();

  ballIntake_mtr.move(65);

  flyWheel(-80);

  pros::delay(1200);

  resetPos();

  //moves forward to the low flag
  moveDrive(28, 63, -91, 0, 0);

  ballIntake_mtr.move(127);

  flyWheel(-91);

  pros::delay(1200);

  driveMotors(-50);

  pros::delay(200);

  resetPos();

  //toggles low flag
  moveDrive(14, 62, 0, 0, 0);

  moveDrive(0, -77, 0, 0, 0);


  driveMotors1(-77);

  pros::delay(200);

  //turns
  driveMotors(50);

  pros::delay(150);

  resetPos();

  //turns towards cap
  driveMotors(50);

  pros::delay(800);

  resetPos();

  // resets alignment with the wall so that it is consistently able to flip the cap
  driveMotors1(-77);

  pros::delay(900);

  driveMotors1(0);

  pause();

  resetPos();

  //flips cap
  moveDrive(29, 67, 0, 0, -50);

  moveDrive(0, -77, 0, 0, 0);

  moveDrive(6, 77, 0, 0, 0);

  // comes back and turns towards the direction of the back of the field
  driveTurn(90, 70);

  pros::delay(1000);

  resetPos();

  //moves to the back of the field
  moveDrive(70, 114, 0, 0, 0);

  pros::delay(200);

  driveTurn(-90, -77);

  pros::delay(1000);

  resetPos();

  driveMotors1(-67);

  pros::delay(800);

  resetPos();

  moveDrive(42, 87, 0, 0, 0);

  moveDrive(52, 107, 0, 0, -60);

  pause();

  ballIntake_mtr.move(127);

  pros::delay(650);

  moveDrive(48, -67, 0, 0, 0);

  driveTurn(90, 77);

  pros::delay(1000);

  resetPos();

  moveDrive(10, 99, 0, 0, 0);

  pause();

  driveTurn(-82, -77);

  pros::delay(1000);

  resetPos();

  moveDrive(58, 77, 0, 0, 0);

  driveTurn(-90, -77);

  pros::delay(1000);

  resetPos();

  driveMotors1(-65);

  pros::delay(1400);

  resetPos();

  moveDrive(70, 100, -91, 0, 0);

  ballIntake_mtr.move(127);

  flyWheel(-91);

  pros::delay(1000);

  driveTurn(-15, -77);

  pros::delay(300);

  resetPos();

  moveDrive(30, 87, 0, 0, -90);

  moveDrive(3, -87, 0, 0, -90);

  driveTurn(-75, -67);

  pros::delay(800);

  resetPos();

  driveMotors1(-77);

  pros::delay(1500);

  moveDrive(7, 67, -95, 0, 0);

  pause();

  driveTurn(-90, -77);

  pros::delay(1200);

  resetPos();

  moveDrive(22, 67, 0, 0, 0);

  pause();

  driveTurn(90, 77);

  pros::delay(1000);

  driveMotors1(-80);

  pros::delay(1000);

  resetPos();

  moveDrive(95, 87, 0, 0, 127);

  resetPos();

}

void blueFrontAuton()
{
  pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor chainBar_mtr(8, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

  resetPos();

  //Moves forward
  moveDrive(40, 107, -91, 0, 0);

  //flips cap
  moveDrive(46, 127, -91, 0, 20);

  ballIntake_mtr.move(60);

  pros::delay(575);

  //moves back
  moveDrive(0, -100, -82, 0, 40);

  resetPos();

  pause();

  driveMotors1(-87);

  pros::delay(300);

  resetPos();

  pause();

  //moves a bit forward
  moveDrive(7, 67, -82, 0, 30);

  pause();

  //turns
  driveTurn(90, 80);

  pros::delay(750);

  resetPos();

  ballIntake_mtr.move(95);

  flyWheel(-80);

  pros::delay(910);

  resetPos();

  //moves forward to the low flag
  moveDrive(27, 93, -91, 0, 0);

  ballIntake_mtr.move(127);

  flyWheel(-91);

  pros::delay(1100);

  driveMotors(50);

  pros::delay(200);

  resetPos();

  //toggles low flag
  moveDrive(14, 62, 0, 0, 0);

  moveDrive(0, -77, 0, 0, 0);


  driveMotors1(-97);

  pros::delay(200);

  driveMotors(-67);

  pros::delay(90);

  resetPos();

  moveDrive(40, -110, 0, 0, 0);

  pause();

  driveTurn(-90, -77);

  pros::delay(900);

  resetPos();

  driveMotors1(-97);

  pros::delay(600);

  resetPos();

  moveDrive(57, 100, 0, 0, 90);

}

void redFrontAuton()
{
  pros::Motor flyLWheel_mtr(5, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor flyRWheel_mtr(6, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);
  pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);
  pros::Motor chainBar_mtr(8, MOTOR_GEARSET_18, false, MOTOR_ENCODER_ROTATIONS);

  resetPos();

  //Moves forward
  moveDrive(40, 107, -91, 0, 0);

  //flips cap
  moveDrive(46, 127, -91, 0, 20);

  ballIntake_mtr.move(60);

  pros::delay(575);

  //moves back
  moveDrive(0, -100, -82, 0, 40);

  resetPos();

  pause();

  driveMotors1(-87);

  pros::delay(300);

  resetPos();

  pause();

  //moves a bit forward
  moveDrive(7, 67, -82, 0, 30);

  pause();

  //turns
  driveTurn(-90, -80);

  pros::delay(750);

  resetPos();

  ballIntake_mtr.move(95);

  flyWheel(-80);

  pros::delay(910);

  resetPos();

  //moves forward to the low flag
  moveDrive(27, 93, -91, 0, 0);

  ballIntake_mtr.move(127);

  flyWheel(-91);

  pros::delay(1100);

  driveMotors(-50);

  pros::delay(200);

  resetPos();

  //toggles low flag
  moveDrive(14, 62, 0, 0, 0);

  moveDrive(0, -77, 0, 0, 0);


  driveMotors1(-97);

  pros::delay(200);

  driveMotors(67);

  pros::delay(90);

  resetPos();

  moveDrive(40, -110, 0, 0, 0);

  pause();

  driveTurn(90, 77);

  pros::delay(900);

  resetPos();

  driveMotors1(-97);

  pros::delay(600);

  resetPos();

  moveDrive(57, 100, 0, 0, 90);
}

void blueBackAuton()
{

}

void redBackAuton()
{

}

void visionSense(int speed)
{
  pros::Vision vision (9);
  pros::Motor ballIntake_mtr(7, MOTOR_GEARSET_18, true, MOTOR_ENCODER_ROTATIONS);

  pros::vision_object_s_t obj = vision.get_by_sig(0, sig);

  if(obj.signature == sig)
  {
    while(obj.signature == sig)
    {
      flyWheel(-107);
      ballIntake_mtr.move(127);
    }
    while(obj.signature == sig)
    {
      flyWheel(-100);
    }
  }
  else
  {
    if(sig == 1)
    {
      driveTurn(-15, -67);
      pros::delay(400);
    }
    else if(sig == 2)
    {
      driveTurn(15, 67);
      pros::delay(400);
    }
  }
}

void autonomous()
{
  skillsAuton();
}
