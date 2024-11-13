// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    var swerve = new Swerve(this);
    var arm = new Mekanism(this);

    waitForStart();
    while (opModeIsActive()) {
      swerve.teleopDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
      swerve.periodic();

      // game pad 2
      // sets arm
      arm.setSlide(-gamepad2.left_stick_y);
      arm.setPivot(gamepad2.right_stick_y, gamepad2.left_bumper);

      arm.moveClaw(gamepad2.right_trigger > .5, gamepad2.left_trigger > .5);
      arm.moveWrist(gamepad2.b);

      telemetry.update();
    }
  }
}
