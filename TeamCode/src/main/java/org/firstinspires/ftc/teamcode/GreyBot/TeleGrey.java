// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.GreyBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Utils;

//@TeleOp(name = "GreyBot", group = "grey")
//@Disable
public class
TeleGrey extends LinearOpMode {

  @Override
  public void runOpMode() {

    Swerve swerve = new Swerve(this);
    Mekanism mek = new Mekanism(this);

    waitForStart();
    mek.arm.homeArm();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {
      double currentTime = Utils.getTimeSeconds();
      double dt = currentTime - lastTime;

      // Updates power to all of the swerve stuff
      swerve.teleopDrive(
          -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, dt);
      swerve.periodic();

      lastTime = currentTime; // Delta time update

      telemetry.update();
    }
  }
}
