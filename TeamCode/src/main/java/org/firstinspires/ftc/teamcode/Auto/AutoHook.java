// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto Hook Specimen")
public class AutoHook extends LinearOpMode {
  private Swerve drivebase;

  @Override
  public void runOpMode() throws InterruptedException {
    drivebase = new Swerve(this);
    drivebase.initGyro();

    waitForStart();
    Rotation2d rotation = new Rotation2d();
    Pose2d target = new Pose2d(1, 0, rotation);
    drivebase.alignWheels(this::opModeIsActive); // TODO: rear wheels just spins idk why
    sleep(1000);
    driveWithOdo(target, 3);

    //to send it a different direction change target and direction and call driveWithOdo with a given time
  }

  public void driveWithOdo(Pose2d wantedPos, double dt) {
    double startTime = Utils.getTimeSeconds();

    ChassisSpeeds speeds = new ChassisSpeeds(wantedPos.getX()/dt, wantedPos.getY()/dt, wantedPos.getRotation().getRadians()/dt);

    while (opModeIsActive()) {
      double currentTime = Utils.getTimeSeconds() - startTime;
      drivebase.drive(speeds, currentTime);
/*
      var currentPos = drivebase.getPose();
      if (currentPos != wantedPos) {   // TODO: find a better way to write this by adding a class into Pose2d that allows for comparison of 2 positions in a dead zone
        var movement = currentPos.minus(wantedPos); // TODO: figure out if this actually works with translating the robot (I think it does)
        ChassisSpeeds newSpeeds = new ChassisSpeeds(movement.getX() / (dt / 2), movement.getY() / (dt / 2), movement.getRotation().getRadians() / (dt / 2));
        driveForTime(newSpeeds, currentTime / 2);
      } else return;
*/
      if (Utils.getTimeSeconds() - startTime >= dt) {
        return;
      }
    }
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

  private void hookClip() {
  }
}
