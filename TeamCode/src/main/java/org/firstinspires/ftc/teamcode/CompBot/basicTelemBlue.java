// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.MathUtil;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.util.Units;

@TeleOp(name = "basic telemetry for blue robot", group = "CompBot")
public class basicTelemBlue extends LinearOpMode {
  /*
  The point of this code is to be a basic telem that works for the blue robot
  date last updated and tested:
  */

  DcMotor FLMotor, BLMotor, BRMotor, FRMotor, pivot, slide, slide2;
  Servo FLServo, BLServo, BRServo, FRServo, intakeL, wrist;

  GoBildaPinpointDriver odo;

  double limitSlide;
  double limitPivot;

  DigitalChannel limitSwitch;

  double pubLength = 0;

  double encoderCountsPerInch = 85;   // needs adjusting

  double encoderCountsPerDegree = 30; // needs adjusting

  static double TRACKWIDTH = 14; // in inches
  static double WHEELBASE = 15; // in inches

  public void runOpMode() throws InterruptedException {

    initRobot(); // initializes the robot

    /**
     * controls for game pad 1:
     * right trigger: forwards
     * left trigger: backwards
     * right stick x: rotate
     * left stick x: strafe
     *
     * <p>controls for game pad 2: left stick y: in and out of arm right stick y: up and down of arm
     * a: constant in changed to out while button is being held while b is being held right stick x:
     * wrist position presets for: attaching clip to sample attaching specimen(clip + sample) to top
     * rung presets for bucket 1 and 2
     */
    waitForStart();

    // waits for start then sets all of the servos
    FLServo.setPosition(0.50);
    BLServo.setPosition(0.50);
    FRServo.setPosition(0.50);
    BRServo.setPosition(0.50);

    wrist.setPosition(0.35);

    while (opModeIsActive()) {
      // game pad 1
      double forBack = -gamepad1.left_stick_y;
      double rotate = gamepad1.right_stick_x;
      double strafe = -gamepad1.left_stick_x;

      if (forBack != 0) {
        forwardBackward(forBack);
      } else if (rotate != 0) {
        rotate(rotate);
      } else if (strafe != 0) {
        strafe(strafe);
      } else {
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
        FRMotor.setPower(0);
      }

      // game pad 2
      slideLimit();
      setSlide(-gamepad2.right_stick_y);
      setPivot(gamepad2.left_stick_y);

      if (gamepad2.right_trigger != 0) {
        intakeL.setPosition(1);
      } else if (gamepad2.left_trigger != 0) {
        intakeL.setPosition(0);
      } else {
        intakeL.setPosition(0.5);
      }

      if (gamepad2.b) {
        double x;
        if (wrist.getPosition() >= .5) x = .35;
        else x = 1;
        wrist.setPosition(x);
      }

      // telemetry stuff (not needed for actual competition, just there for testing purposes)
      telemetry.addData("Pivot encoder count: ", pivot.getCurrentPosition());
      telemetry.addData("Slide encoder count: ", slide.getCurrentPosition());
      telemetry.update();
    }
  }

  public void topBucketPreset() {
    setPivot(90 - 5 * encoderCountsPerDegree);
    //setSlide();
  }

  public void toTopSpecimenRung() {
    setPivot(90 - 5 * encoderCountsPerDegree);
    // setSlide();
  }

  /**
   * Initializes the robot.<br>
   * Starts all the devices and maps where they go As well as whether motors run with encoders or
   * not
   */
  public void initRobot() {

    // Maps the motor objects to the physical ports
    FLMotor =
      hardwareMap.get(
        DcMotor.class, "FLMotor"); // TODO: Run testMotor to figure out which motor is where
    BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
    FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
    BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

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

    FLMotor.setDirection(FORWARD);
    BLMotor.setDirection(REVERSE);
    FRMotor.setDirection(REVERSE);
    BRMotor.setDirection(FORWARD);

    FLMotor.setPower(0);
    BLMotor.setPower(0);
    FRMotor.setPower(0);
    BRMotor.setPower(0);

    // Maps the servo objects to the physical ports
    FLServo = hardwareMap.get(Servo.class, "FLServo");
    BLServo = hardwareMap.get(Servo.class, "BLServo");
    FRServo = hardwareMap.get(Servo.class, "FRServo");
    BRServo = hardwareMap.get(Servo.class, "BRServo");

    // Sets the ends of the servos. Hover cursor over function for more info
    // Will need to be tuned later
    FLServo.scaleRange(0, 1.0);
    BLServo.scaleRange(0, 1.0);
    FRServo.scaleRange(0, 1.0);
    BRServo.scaleRange(0, 1.0);

    // Init GoBilda Pinpoint module
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.resetPosAndIMU();
    odo.setOffsets(177.8, 50.8);
    odo.setEncoderDirections(
      GoBildaPinpointDriver.EncoderDirection.FORWARD,
      GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Init slaw, claw, and pivot
    pivot = hardwareMap.dcMotor.get("pivot");
    slide = hardwareMap.dcMotor.get("slide");
    slide2 = hardwareMap.dcMotor.get("slide 2");

    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);
    slide2.setMode(STOP_AND_RESET_ENCODER);

    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_USING_ENCODER);
    slide2.setMode(RUN_USING_ENCODER);

    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide2.setZeroPowerBehavior(BRAKE);

    pivot.setDirection(FORWARD);
    slide.setDirection(FORWARD);
    slide2.setDirection(FORWARD);

    pivot.setPower(0);
    slide.setPower(0);
    slide2.setPower(0);

    limitSlide = 4500;
    limitPivot = 2750;

    limitSwitch = hardwareMap.get(DigitalChannel.class, "limit switch");

    // servos for intake
    intakeL = hardwareMap.get(Servo.class, "intake");
    wrist = hardwareMap.get(Servo.class, "wrist");

    intakeL.setDirection(Servo.Direction.REVERSE);
    wrist.setDirection(Servo.Direction.FORWARD);
  }

  // to lift arm, input from game pad 2 straight in
  public void setSlide(double x) {
    if (slide.getCurrentPosition() >= limitSlide && x > 0) {
      x = 0;
    } else if (slide.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }

    if (x == 0)
      x =
        MathUtil.interpolate(
          .00125,
          .005,
          MathUtil.inverseInterpolate(0, limitSlide, slide.getCurrentPosition()))
          * Math.sin(
          Units.degreesToRadians(
            90 - (pivot.getCurrentPosition() * encoderCountsPerDegree)));

    if (x > 0 && slide.getCurrentPosition() > pubLength) {
      x = 0;
    }

    slide.setPower(x);
    slide2.setPower(x);
  }

  public void slideLimit() {
    pubLength =
      Math.cos(Math.toRadians(pivot.getCurrentPosition() / encoderCountsPerDegree))
        * (46 * encoderCountsPerInch);
    if (pubLength <= 2950) pubLength = 2950;
    if(pivot.getCurrentPosition()/encoderCountsPerDegree <= 7.5 )
      pubLength = limitSlide;
  }

  public void setPivot(double x) {
    if (gamepad2.left_bumper) limitPivot += 500;
    if (pivot.getCurrentPosition() >= limitPivot && x > 0) {
      x = 0;
    } else if (pivot.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }

    if (x > 1) x = .5;
    if (x < -1) x = -.5;

    if (slide.getCurrentPosition() > pubLength && x > 1) slide.setPower(-x * 2);
    pivot.setPower(x);

    if (limitPivot > 2750) limitPivot = 2750;
  }

  /**
   * Converts standard cartesian coordinates to polar coordinates not used (yet)
   *
   * @param x X input
   * @param y Y input
   * @return Returns an array of two doubles,<br>
   * <p>[0] = R - Magnitude<br>
   * [1] = Theta Angle of input coordinate.<br>
   * Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
   * @see #polarToCartesian(double, double)
   */
  public double[] cartesianToPolar(double x, double y) {
    double[] arrayToReturn = new double[2];
    arrayToReturn[0] = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Radius
    arrayToReturn[1] = Math.atan(y / x) * (Math.PI / 180); // Theta

    return arrayToReturn;
  }

  /**
   * Converts polar coordinates to cartesian not used (yet)
   *
   * @param r     Magnitude of input coordinate
   * @param theta Angle of input coordinate.<br>
   *              Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
   * @return Returns an array of two doubles,<br>
   * <p>[0] = X<br>
   * [1] = Y
   * @see #cartesianToPolar(double, double)
   */
  public double[] polarToCartesian(double r, double theta) {
    double[] arrayToReturn = new double[2];
    arrayToReturn[0] = r * Math.cos(theta); // X
    arrayToReturn[1] = r * Math.sin(theta); // Y

    return arrayToReturn;
  }

  /**
   * Moves the robot forward/backward based on desired power.<br>
   * Does not change the rotation of the robot at all
   *
   * @param power Desired power to run the motors at
   */
  public void forwardBackward(double power) {
    FLServo.setPosition(0.5);
    FRServo.setPosition(0.5);
    BLServo.setPosition(0.5);
    BRServo.setPosition(0.5);

    FLMotor.setPower(power);
    BLMotor.setPower(power);
    BRMotor.setPower(power);
    FRMotor.setPower(power);
  }

  public void strafe(double power) {
    // Set wheels for strafe
    FLServo.setPosition(.80);
    BLServo.setPosition(.20);
    BRServo.setPosition(.80);
    FRServo.setPosition(.20);

    // turn motors to strafe robot
    FLMotor.setPower(power); // actually BR
    BLMotor.setPower(power);
    BRMotor.setPower(-power);
    FRMotor.setPower(-power); // actually BL
  }

  /**
   * Rotates the robot around itself.<br>
   * Does not move the robot in any other direction.<br>
   *
   * @param power Desired power to turn the robot at
   */
  public void rotate(double power) {
    // Set wheels for rotation
    FLServo.setPosition(.35);
    BLServo.setPosition(.60);
    BRServo.setPosition(.60);
    FRServo.setPosition(.35);

    // turn motors to strafe robot
    FLMotor.setPower(-power);
    BLMotor.setPower(power);
    BRMotor.setPower(-power);
    FRMotor.setPower(power);
  }

  /**
   * TODO this needs to be worked on nothing is done here
   *
   * <p>Wheel angle is perpendicular to turn angle turn angle is inverse tan(get angle) of 7.5(half
   * of wheel base length) / (turning distance/2) because it is radius of point we are trying to
   * rotate around speed = -1 to 1 turnRad = -1 to 1 turnDir = LEFT or RIGHT
   */
  public void moveAndRotate(double speed, double turnAmount) {

    double i_WheelAngle = Math.atan2(WHEELBASE, turnAmount - TRACKWIDTH / 2);
    double outsideAng = Math.atan2(WHEELBASE, turnAmount + TRACKWIDTH / 2);
  }

  /**
   * TODO Possibly make this not run at full speed at all times by adding some sort of power input
   * Uses the IMU to move the robot to face forward
   *
   * <p>As long as this function is called, it will try to rotate back to facing forward
   */
  public void rotateToCenter() {
    double orientation = odo.getHeading().getRadians();
    telemetry.addData("Yaw angle", orientation);

    rotate(orientation);
  }

  /**
   * TODO All of this Moves the robot to the detected specimen
   */
  public void moveToSpecimen() {
  }

  /**
   * TODO All of this as well Moves the robot back to the storage area
   */
  public void moveToStore() {
  }
}
