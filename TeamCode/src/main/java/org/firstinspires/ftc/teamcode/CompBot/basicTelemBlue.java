
package org.firstinspires.ftc.teamcode.CompBot;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;

@TeleOp(name = "basic telemetry for blue robot", group = "CompBot")
public class basicTelemBlue extends LinearOpMode {
    /*
    The point of this code is to be a basic telem that works for the blue robot
    date last updated and tested:
    */

  DcMotor FLMotor, BLMotor, BRMotor, FRMotor, pivot, slide;

  Servo FLServo, BLServo, BRServo, FRServo, intakeL, wrist;

  GoBildaPinpointDriver odo;


  ElapsedTime turnTime = new ElapsedTime();


  // In case we need to add it later, but servos only have 180° so have to be perfectly placed
  double FLServoOffSet = .00;
  double FRServoOffSet = .00;
  double BLServoOffSet = .00;
  double BRServoOffSet = .00;


  static double TRACKWIDTH = 14;      //in inches
  static double WHEELBASE = 15;       //in inches


  public void runOpMode() throws InterruptedException {

    initRobot(); // Does all the robot stuff

    /**
     * controls for game pad 1:
     * right trigger: forwards
     * left trigger: backwards
     * right stick x: rotate
     * left stick x: strafe
     *
     *
     * controls for game pad 2:
     * left stick y: in and out of arm
     * right stick y: up and down of arm
     * a: constant in changed to out while button is being held
     * while b is being held right stick x: wrist position
     * presets for:
     * attaching clip to sample
     * attaching specimen(clip + sample) to top rung
     * presets for bucket 1 and 2
     */

    waitForStart();
    while (opModeIsActive()) {

      //game pad 1
      double forBack = -gamepad1.left_stick_y; // Makes it so that the triggers cancel each other out if both are pulled at the same time
      double rotate = gamepad1.right_stick_x;

      if (forBack != 0) forwardBackward(forBack);
      else if (rotate != 0) rotate(rotate);
      else {
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);
        FRMotor.setPower(0);
      }


      //game pad 2 inputs
      double armLength = -gamepad2.right_stick_y;
      double armAngle = -gamepad2.left_stick_y;

      //call move arm
      extendArm(armLength);
      liftArm(armAngle);

      //claw intake/outtake
      if (gamepad2.right_trigger != 0) {
        intakeL.setPosition(1);
      }
      else if (gamepad2.left_trigger != 0) {
        intakeL.setPosition(0);
      }
      else
        intakeL.setPosition(0.5);

      //wrist rotation
      if (gamepad2.b) {
        double x;
        if (wrist.getPosition() >= .1)
          x = 0;
        else
          x = .7;
        wrist.setPosition(x);
      }
    }
  }


  // to lift arm, input from game pad 2 straight in
  public void liftArm(double x) {
    if (pivot.getCurrentPosition() >= 2500 && x > 0) {
      x = 0;
    } else if (pivot.getCurrentPosition() <= -2500 && x < 0) {
      x = 0;
    }
    pivot.setPower(x);
    telemetry.addData("lift power: ", pivot.getPower());
  }

  // to extend arm, input from game pad 2 straight in
  public void extendArm(double x) {
    if (slide.getCurrentPosition() >= 4750 && x > 0) {
      x = 0;
    } else if (slide.getCurrentPosition() <= -4750 && x < 0) {
      x = 0;
    }
    slide.setPower(x);
    telemetry.addData("slide power: ", slide.getPower());
  }


  /**
   * Initializes the robot.<br>
   * Starts all the devices and maps where they go
   * As well as whether motors run with encoders or not
   */
  public void initRobot() {

    // Maps the motor objects to the physical ports
    FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
    BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
    FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
    BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

    // Sets the encoder mode
    FLMotor.setMode(RUN_USING_ENCODER);
    BLMotor.setMode(RUN_USING_ENCODER);
    FRMotor.setMode(RUN_USING_ENCODER);
    BRMotor.setMode(RUN_USING_ENCODER);

    // Sets what happens when no power is applied to the motors.
    // In this mode, the computer will short the 2 leads of the motor, and because of math, the motor will be a lot harder to turn
    FLMotor.setZeroPowerBehavior(BRAKE);
    BLMotor.setZeroPowerBehavior(BRAKE);
    FRMotor.setZeroPowerBehavior(BRAKE);
    BRMotor.setZeroPowerBehavior(BRAKE);

    FLMotor.setDirection(FORWARD);
    BLMotor.setDirection(FORWARD);
    FRMotor.setDirection(FORWARD);
    BRMotor.setDirection(REVERSE);


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

    FLServo.setPosition(0.50);
    BLServo.setPosition(0.50);
    FRServo.setPosition(0.50);
    BRServo.setPosition(0.50);


    // Init GoBilda Pinpoint module
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.resetPosAndIMU();
    odo.setOffsets(177.8, 50.8);
    odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


    // Init slaw, claw, and pivot
    pivot = hardwareMap.dcMotor.get("pivot");
    slide = hardwareMap.dcMotor.get("slide");

    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_USING_ENCODER);

    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    pivot.setDirection(REVERSE);
    slide.setDirection(FORWARD);

    pivot.setPower(0);
    slide.setPower(0);


    //servos for intake
    intakeL = hardwareMap.get(Servo.class, "intake");
    wrist = hardwareMap.get(Servo.class, "wrist");

    intakeL.setDirection(Servo.Direction.REVERSE);
    wrist.setDirection(Servo.Direction.FORWARD);

    wrist.setPosition(0.7);
  }


  /**
   * Converts standard cartesian coordinates to polar coordinates
   * not used (yet)
   *
   * @param x X input
   * @param y Y input
   * @return Returns an array of two doubles,<br>
   * <p>
   * [0] = R - Magnitude<br>
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
   * Converts polar coordinates to cartesian
   * not used (yet)
   *
   * @param r     Magnitude of input coordinate
   * @param theta Angle of input coordinate.<br>
   *              Relative to unit circle, where 0deg in is to the right, 90 is up and 180 is left.
   * @return Returns an array of two doubles,<br>
   * <p>
   * [0] = X<br>
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
   * @param power   Desired power to run the motors at
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


  /**
   * Rotates the robot around itself.<br>
   * Does not move the robot in any other direction.<br>
   *
   * @param power Desired power to turn the robot at
   */
  public void rotate(double power) {

    // Set wheels for rotation
    FLServo.setPosition(.80);
    BLServo.setPosition(.20);
    BRServo.setPosition(.80);
    FRServo.setPosition(.20);

    //turn motors to rotate robot
    FLMotor.setPower(power);
    BLMotor.setPower(power);
    BRMotor.setPower(-power);
    FRMotor.setPower(-power);
  }


  /**
   * TODO this needs to be worked on nothing is done here
   *
   * <p>
   * Wheel angle is perpendicular to turn angle
   * turn angle is inverse tan(get angle) of 7.5(half of wheel base length) / (turning distance/2)
   * because it is radius of point we are trying to rotate around
   * speed = -1 to 1
   * turnRad = -1 to 1
   * turnDir = LEFT or RIGHT
   */
  public void moveAndRotate(double speed, double turnAmount) {

    double i_WheelAngle = Math.atan2(WHEELBASE, turnAmount - TRACKWIDTH / 2);
    double outsideAng = Math.atan2(WHEELBASE, turnAmount + TRACKWIDTH / 2);


  }


  /**
   * TODO Possibly make this not run at full speed at all times by adding some sort of power input
   * Uses the IMU to move the robot to face forward
   * <p>
   * As long as this function is called, it will try to rotate back to facing forward
   */
  public void rotateToCenter() {
    double orientation = odo.getHeading();
    telemetry.addData("Yaw angle", orientation);

    rotate(orientation);
  }


  /**
   * TODO All of this
   * Moves the robot to the detected specimen
   */
  public void moveToSpecimen() {

  }


  /**
   * TODO All of this as well
   * Moves the robot back to the storage area
   */
  public void moveToStore() {

  }

}