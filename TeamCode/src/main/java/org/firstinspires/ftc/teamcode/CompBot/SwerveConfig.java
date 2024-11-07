// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

public class SwerveConfig {

  public LinearOpMode myOp;

  SwerveConfig(LinearOpMode opMode) {
    myOp = opMode;
  }

  DcMotor FLMotor, BLMotor, FRMotor, BRMotor;
  Servo FLServo, BLServo, FRServo, BRServo;

  GoBildaPinpointDriver odo;

  /** TODO Get rid of this somehow */
  // In case builders are bad, is offset center for servo
  double FL_SERVO_OFFSET = .005;

  double BL_SERVO_OFFSET = .01;
  double FR_SERVO_OFFSET = .00;
  double BR_SERVO_OFFSET = 0.007;

  // Robot dimensions
  static double TRACKWIDTH = 14;
  static double WHEELBASE = 15;

  /** Initializes all of the components related to the swerve drive */
  public void initSwerve() {

    // Maps the motor objects to the physical ports
    FLMotor = myOp.hardwareMap.get(DcMotor.class, "FLMotor");
    BLMotor = myOp.hardwareMap.get(DcMotor.class, "BLMotor");
    FRMotor = myOp.hardwareMap.get(DcMotor.class, "FRMotor");
    BRMotor = myOp.hardwareMap.get(DcMotor.class, "BRMotor");

    // Sets the encoder mode
    FLMotor.setMode(RUN_USING_ENCODER);
    BLMotor.setMode(RUN_USING_ENCODER);
    FRMotor.setMode(RUN_USING_ENCODER);
    BRMotor.setMode(RUN_USING_ENCODER);

    // Sets what happens when no power is applied to the motors.
    // In this mode, the computer will short the 2 leads of the motor, and because of math, the
    // motor will be a lot harder to turn
    FLMotor.setZeroPowerBehavior(BRAKE);
    BLMotor.setZeroPowerBehavior(BRAKE);
    FRMotor.setZeroPowerBehavior(BRAKE);
    BRMotor.setZeroPowerBehavior(BRAKE);

    FLMotor.setDirection(REVERSE);
    BLMotor.setDirection(REVERSE);
    FRMotor.setDirection(REVERSE);
    BRMotor.setDirection(FORWARD);

    // Maps the servo objects to the physical ports
    FLServo = myOp.hardwareMap.get(Servo.class, "FLServo");
    BLServo = myOp.hardwareMap.get(Servo.class, "BLServo");
    FRServo = myOp.hardwareMap.get(Servo.class, "FRServo");
    BRServo = myOp.hardwareMap.get(Servo.class, "BRServo");

    // Sets the ends of the servos. Hover cursor over function for more info
    // Will need to be tuned later
    FLServo.scaleRange(FL_SERVO_OFFSET, 1.0 + FL_SERVO_OFFSET * 2);
    BLServo.scaleRange(BL_SERVO_OFFSET, 1.0 + BL_SERVO_OFFSET * 2);
    FRServo.scaleRange(FR_SERVO_OFFSET, 1.0 + FR_SERVO_OFFSET * 2);
    BRServo.scaleRange(BR_SERVO_OFFSET, 1.0 + BR_SERVO_OFFSET * 2);

    setFLServo(0);
    setBLServo(0);
    setFRServo(0);
    setBRServo(0);

    // Init GoBilda Pinpoint module
    odo = myOp.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.resetPosAndIMU();
    odo.setOffsets(177.8, 50.8);
    odo.setEncoderDirections(
        GoBildaPinpointDriver.EncoderDirection.FORWARD,
        GoBildaPinpointDriver.EncoderDirection.FORWARD);
  }

  /**
   * Moves the robot based on desired heading and power.
   *
   * <p>Only moves in a straight line. Does not change the rotation of the robot at all
   *
   * @param heading Desired heading of the robot.<br>
   *     0 is straight forward 1 is fully right, -1 is fully left
   * @param power Desired power to run the motors at
   */
  public void moveStraight(double heading, double power) {
    heading = (heading + 1) / 2; // Converts the -1 to 1 input range to 0 to 1 for the servos

    FLMotor.setPower(power);
    BLMotor.setPower(power);
    BRMotor.setPower(power);
    FRMotor.setPower(power);

    FLServo.setPosition(heading + FL_SERVO_OFFSET);
    BLServo.setPosition(heading + BL_SERVO_OFFSET);
    BRServo.setPosition(heading + BR_SERVO_OFFSET);
    FRServo.setPosition(heading + FR_SERVO_OFFSET);
  }

  /**
   * Rotates the robot around a center point.<br>
   * Does not move the robot in any other direction.<br>
   *
   * @param power Power to turn the robot at
   */
  public void rotate(double power) {

    // turn motors to rotate robot
    FLMotor.setPower(-power);
    BLMotor.setPower(-power);
    BRMotor.setPower(power);
    FRMotor.setPower(power);

    // Set wheels for rotation (Ben's robot has 2x gear ratio so .25/2 and .75/2)
    FLServo.setPosition(.25 + .125 / 2);
    BLServo.setPosition(.75 - .125 / 2);
    BRServo.setPosition(.25 + .125 / 2);
    FRServo.setPosition(.75 - .125 / 2);
  }

  /**
   * TODO this needs to be worked on nothing is done here
   *
   * <p>Wheel angle is perpendicular to turn angle turn angle is inverse tan(get angle) of 7.5(half
   * of wheel base length) / (turning distance/2) because it is radius of point we are trying to
   * rotate around speed = -1 to 1 turnRad = -1 to 1 turnDir = LEFT or RIGHT
   */
  public void moveAndRotate(double power, double turn) {

    double i_WheelAngle = Math.atan2(WHEELBASE, turn - TRACKWIDTH / 2);
    double outsideAng = Math.atan2(WHEELBASE, turn + TRACKWIDTH / 2);
  }

  /**
   * TODO Possibly make this not run at full speed at all times by adding some sort of power input
   * Uses the IMU to move the robot to face forward
   *
   * <p>As long as this function is called, it will try to rotate back to facing forward
   */
  public void centerRobot() {
    double orientation = odo.getHeading().getRadians();

    rotate(orientation);
  }

  /**
   * All of the set__Servo functions set the position of the servo using a -1 to 1 range
   *
   * @param pos (-1) to 1 input
   * @see #setFLServo(double)
   * @see #setBLServo(double)
   * @see #setFRServo(double)
   * @see #setBRServo(double)
   */
  public void setFLServo(double pos) {

    pos = (pos + 1) / 2; // Converts the -1 to 1 input range to 0 to 1 for the servos

    FLServo.setPosition(pos + FL_SERVO_OFFSET);
  }

  /**
   * All of the set__Servo functions set the position of the servo using a -1 to 1 range
   *
   * @param pos (-1) to 1 input
   * @see #setFLServo(double)
   * @see #setBLServo(double)
   * @see #setFRServo(double)
   * @see #setBRServo(double)
   */
  public void setBLServo(double pos) {
    pos = (pos + 1) / 2; // Converts the -1 to 1 input range to 0 to 1 for the servos

    BLServo.setPosition(pos + BL_SERVO_OFFSET);
  }

  /**
   * All of the set__Servo functions set the position of the servo using a -1 to 1 range
   *
   * @param pos (-1) to 1 input
   * @see #setFLServo(double)
   * @see #setBLServo(double)
   * @see #setFRServo(double)
   * @see #setBRServo(double)
   */
  public void setFRServo(double pos) {
    pos = (pos + 1) / 2; // Converts the -1 to 1 input range to 0 to 1 for the servos

    FRServo.setPosition(pos + FR_SERVO_OFFSET);
  }

  /**
   * All of the set__Servo functions set the position of the servo using a -1 to 1 range
   *
   * @param pos (-1) to 1 input
   * @see #setFLServo(double)
   * @see #setBLServo(double)
   * @see #setFRServo(double)
   * @see #setBRServo(double)
   */
  public void setBRServo(double pos) {
    pos = (pos + 1) / 2; // Converts the -1 to 1 input range to 0 to 1 for the servos

    BRServo.setPosition(pos + BR_SERVO_OFFSET);
  }

  /** TODO All of this Moves the robot to the detected specimen */
  public void moveToSpecimen() {}

  /** TODO All of this as well Moves the robot back to the storage area */
  public void moveToStore() {}
}
