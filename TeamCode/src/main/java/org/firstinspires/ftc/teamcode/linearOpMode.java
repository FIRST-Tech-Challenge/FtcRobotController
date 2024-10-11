package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveTrainTest1 (Blocks to Java)")
public class linearOpMode extends LinearOpMode {

  private DcMotor leftBack;
  private DcMotor leftFront;
  private DcMotor rightBack;
  private DcMotor rightFront;
  private Servo claw;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    claw = hardwareMap.get(Servo.class, "claw");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        leftBack.setPower(-gamepad1.left_stick_y);
        leftFront.setPower(-gamepad1.left_stick_y);
        rightBack.setPower(gamepad1.right_stick_y);
        rightFront.setPower(gamepad1.right_stick_y);
        telemetry.update();
      }
    }
  }
}
