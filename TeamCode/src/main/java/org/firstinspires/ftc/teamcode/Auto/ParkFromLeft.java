// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(preselectTeleOp = "Blue Bot Teleop")
public class ParkFromLeft extends LinearOpMode {
  private Swerve drivebase;

  @Override
  public void runOpMode() throws InterruptedException {
    drivebase = new Swerve(this);
    drivebase.initGyro();
    // drivebase.setGyro(Rotation2d.fromDegrees(120));

    waitForStart();
    driveForTime(new ChassisSpeeds(0, -.75, 0), .5);
    driveForTime(new ChassisSpeeds(0, 0, -1), 1.5);
    driveForTime(new ChassisSpeeds(0, -0.75, 0), .75);
    driveForTime(new ChassisSpeeds(-.75, 0.125, 0), 7);
    driveForTime(new ChassisSpeeds(0, 0.75, 0), 2);
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
