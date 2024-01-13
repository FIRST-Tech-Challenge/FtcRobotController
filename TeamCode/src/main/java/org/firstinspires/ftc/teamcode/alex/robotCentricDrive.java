package org.firstinspires.ftc.teamcode.alex;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "robotZentricDrive (Scorpion)")
@Disabled

public class robotCentricDrive extends LinearOpMode {

  private Servo leftGrip;
  private Servo rightGrip;
  private DcMotor armRotate;
  private DcMotor armExt;
  private DcMotor frontLeftMotor;
  private DcMotor backLeftMotor;
  private DcMotor frontRightMotor;
  private DcMotor backRightMotor;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override

  public void runOpMode() {

    //Initialize Variables
    //Tracks rotation of the arm's motor
    int rotation;
    //Tracks the extension of the arm
    int ext;
    //X and Y values of stick inputs to compile drive outputs
    double y;
    double x;

    leftGrip = hardwareMap.get(Servo.class, "leftGrip");
    rightGrip = hardwareMap.get(Servo.class, "rightGrip");
    armRotate = hardwareMap.get(DcMotor.class, "armRotate");
    armExt = hardwareMap.get(DcMotor.class, "armExt");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

    //Sets initial positions and directions for grip servos
    leftGrip.setDirection(Servo.Direction.REVERSE);
    rightGrip.setDirection(Servo.Direction.FORWARD);

    //Sets variables to 0 on initialization
    rotation = 0;
    ext = 0;
    waitForStart();

    if (opModeIsActive()) {

      //Sets behaviors and modes for motors
        //ArmExtension and ArmRotate are set to brake when receiving zero power
        //Arm Extension is set to run using encoder outputs and inputs
      armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

      while (opModeIsActive()) {

        //Assigns variables to inputs
          //Ext and Rotation are set to receive inputs from encoders
        ext = armExt.getCurrentPosition();
        rotation = armRotate.getCurrentPosition();
        //Multipliers are applied to X and Y and they are tied to sticks on the gamepads
        y = gamepad1.left_stick_y;
        x = gamepad1.right_stick_x;



        //Sets power for driving wheels
        if(y > 0.2 || y < -0.2){
          backLeftMotor.setPower((y-0.1)*-1);
          backRightMotor.setPower(y);
        } else if (x > 0.2) {
          backLeftMotor.setPower(x-0.1);
          backRightMotor.setPower(x);
        } else if (x < -0.2) {
          backLeftMotor.setPower(x-0.1);
          backRightMotor.setPower(x);
        } else {
          backLeftMotor.setPower(0);
          backRightMotor.setPower(0);
        }

        //Arm rotation controls
          //Rotates up when Right Bumper is pressed
          //Rotates down when Left Bumper is pressed
          //Otherwise power is set to 0 (BRAKE)
        if (gamepad2.right_bumper) {
          armRotate.setPower(0.4);
        } else if (gamepad2.left_bumper) {
          armRotate.setPower(-0.2);
        } else {
          armRotate.setPower(0);
        }

        //Arm extension controls
          //Moves up when X is pressed
          //Moves down when B is pressed
          //Otherwise set power to 0 (BRAKE)
        //Limiters are applied if the motors position is less than 350° or greater than 2900°
          //If the position is less than 350°, the arm can only extend forward
          //If the position is greater than 2900°, the arm can only retract
        if (ext > 350 && ext < 2900) {
          if (gamepad2.dpad_up) {
            armExt.setPower(1);
          } else if (gamepad2.dpad_down) {
            armExt.setPower(-1);
          } else {
            armExt.setPower(0);
          }
        } else if (ext < 350 && ext < 2900) {
          if (gamepad2.dpad_up) {
            armExt.setPower(1);
          } else {
            armExt.setPower(0);
          }
        } else if (ext > 2900 && ext > 350) {
          if (gamepad2.dpad_down) {
            armExt.setPower(-1);
          } else {
            armExt.setPower(0);
          }
        } if(gamepad1.right_trigger > 0.5 && gamepad1.left_trigger > 0.5){
          armExt.setPower(-1);
          sleep(1000000);
        }

        //Inputs for claw grip
          //When A is pressed, the claw releases its grip
          //When Y is pressed, the claw moves to a full grip position
        if (gamepad2.a) {
          leftGrip.setPosition(0.75);
          rightGrip.setPosition(0.75);
        } else if (gamepad2.b) {
          leftGrip.setPosition(1);
          rightGrip.setPosition(1);
        }



        //Telemetry for debugging
        telemetry.addData("Current Arm Extension", ext);
        telemetry.addData("Current Arm Rotation", rotation);
        telemetry.addData("Right Wheel Power", backRightMotor.getPower());
        telemetry.addData("Left Wheel Power", backLeftMotor.getPower());
        telemetry.update();
      }
    }
  }
}
