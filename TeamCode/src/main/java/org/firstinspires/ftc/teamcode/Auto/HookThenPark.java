// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Utils;

@Disabled
@Autonomous(preselectTeleOp = "Blue Bot Teleop")
public class HookThenPark extends LinearOpMode {
  private Swerve drivebase;

  @Override
  public void runOpMode() throws InterruptedException {
    drivebase = new Swerve(this);
    drivebase.initGyro();

    waitForStart();
    drivebase.alignWheels(this::opModeIsActive);
    driveForTime(new ChassisSpeeds(.5, 0, 0), 1);
  }

  private void driveForTime(ChassisSpeeds speeds, double time) {
    double startTime = Utils.getTimeSeconds();
    double lastTime = startTime;
    while (opModeIsActive()) {
      double currentTime = Utils.getTimeSeconds();
      drivebase.drive(speeds, currentTime - lastTime);
      if (Utils.getTimeSeconds() - startTime >= time) {
        return;
      }
      lastTime = currentTime;
    }
  }
}
