package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "robotCentricDrive (Blocks to Java)")
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
    int rotation;
    int ext;
    float y;
    double x;

    leftGrip = hardwareMap.get(Servo.class, "leftGrip");
    rightGrip = hardwareMap.get(Servo.class, "rightGrip");
    armRotate = hardwareMap.get(DcMotor.class, "armRotate");
    armExt = hardwareMap.get(DcMotor.class, "armExt");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

    // Put initialization blocks here.
    leftGrip.setDirection(Servo.Direction.REVERSE);
    rightGrip.setDirection(Servo.Direction.FORWARD);
    leftGrip.setPosition(0.75);
    rightGrip.setPosition(0.75);
    rotation = 0;
    ext = 0;
    waitForStart();
    if (opModeIsActive()) {
      armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        ext = armExt.getCurrentPosition();
        rotation = armRotate.getCurrentPosition();
        y = -1 * -gamepad1.left_stick_y;
        x = -0.5 * gamepad1.right_stick_x;
        frontLeftMotor.setPower(y);
        backLeftMotor.setPower(-y);
        frontRightMotor.setPower(y);
        backRightMotor.setPower(y);
        frontRightMotor.setPower(x);
        backRightMotor.setPower(-x);
        frontLeftMotor.setPower(-x);
        backLeftMotor.setPower(-x);
        if (gamepad1.right_bumper) {
          armRotate.setPower(0.4);
        } else if (gamepad1.left_bumper) {
          armRotate.setPower(-0.2);
        } else {
          armRotate.setPower(0);
        }
        if (ext > 350 && ext < 2900) {
          if (gamepad1.x) {
            armExt.setPower(1);
          } else if (gamepad1.b) {
            armExt.setPower(-1);
          } else {
            armExt.setPower(0);
          }
        } else if (ext < 350 && ext < 2900) {
          if (gamepad1.x) {
            armExt.setPower(1);
          } else {
            armExt.setPower(0);
          }
        } else if (ext > 2900 && ext > 350) {
          if (gamepad1.b) {
            armExt.setPower(-1);
          } else {
            armExt.setPower(0);
          }
        } else {
          armExt.setPower(null);
        }
        if (gamepad1.a) {
          leftGrip.setPosition(0.75);
          rightGrip.setPosition(0.75);
        } else if (gamepad1.y) {
          leftGrip.setPosition(1);
          rightGrip.setPosition(1);
        }
        telemetry.addData("Current Arm Extension", ext);
        telemetry.addData("Current Arm Rotation", rotation);
        telemetry.update();
      }
    }
  }
}
