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
    while (opModeIsActive()) {
      fl.setPower(gamepad1.x ? 1 : 0);
      fr.setPower(gamepad1.y ? 1 : 0);
      bl.setPower(gamepad1.a ? 1 : 0);
      br.setPower(gamepad1.b ? 1 : 0);

      telemetry.addData("FLSpeed", fl.getVelocity());
      telemetry.addData("FRSpeed", fr.getVelocity());
      telemetry.addData("BLSpeed", bl.getVelocity());
      telemetry.addData("BRSpeed", br.getVelocity());

      telemetry.update();
    }
  }
}
