// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.GreyBot;

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
    FLServo.scaleRange(0.0, 1.0);
    BLServo.scaleRange(0.0, 1.0);
    FRServo.scaleRange(0.0, 1.0);
    BRServo.scaleRange(0.0, 1.0);

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
   * All of the set__Servo functions set the position of the servo using a -1 to 1 range
   *
   * @param pos (-1) to 1 input
   * @see #setFLServo(double)
   * @see #setBLServo(double)
   * @see #setFRServo(double)
   * @see #setBRServo(double)
   */
  public void setFLServo(double pos) {
    FLServo.setPosition((pos + 1) / 2); // Converts the -1 to 1 input range to 0 to 1 for the servos
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
    BLServo.setPosition((pos + 1) / 2); // Converts the -1 to 1 input range to 0 to 1 for the servos
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
    FRServo.setPosition((pos + 1) / 2); // Converts the -1 to 1 input range to 0 to 1 for the servos
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
    BRServo.setPosition(pos); // Converts the -1 to 1 input range to 0 to 1 for the servos
  }
}
