// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class DriveMotorTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    DcMotorEx fl = (DcMotorEx) hardwareMap.dcMotor.get("FLMotor");
    DcMotorEx fr = (DcMotorEx) hardwareMap.dcMotor.get("FRMotor");
    DcMotorEx bl = (DcMotorEx) hardwareMap.dcMotor.get("BLMotor");
    DcMotorEx br = (DcMotorEx) hardwareMap.dcMotor.get("BRMotor");

    waitForStart();
    boolean leftPressed = false;
    boolean rightPressed = false;
    double power = 1;
    while (opModeIsActive()) {
      if (gamepad1.left_bumper && !leftPressed) {
        power += .01;
      }
      leftPressed = gamepad1.left_bumper;
      if (gamepad1.right_bumper && !rightPressed) {
        power -= .01;
      }
      rightPressed = gamepad1.right_bumper;

      fl.setPower(gamepad1.x ? power : 0);
      fr.setPower(gamepad1.y ? power : 0);
      bl.setPower(gamepad1.a ? power : 0);
      br.setPower(gamepad1.b ? power : 0);

      telemetry.addData("Power", power);
      telemetry.addData("FLSpeed", fl.getVelocity());
      telemetry.addData("FRSpeed", fr.getVelocity());
      telemetry.addData("BLSpeed", bl.getVelocity());
      telemetry.addData("BRSpeed", br.getVelocity());

      telemetry.update();
    }
  }
}
