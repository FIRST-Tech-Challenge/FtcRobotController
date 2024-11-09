// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

/*
 *-* Control configuration
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Swerve Automation", group = "Swerve")
public class SwerveTeleOp extends LinearOpMode {
  @Override
  public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);

    var swerve = new Swerve(this);

    waitForStart();
    while (opModeIsActive()) {
      swerve.teleopDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
      swerve.periodic();
      telemetry.update();
    }
  }
}
