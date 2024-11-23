package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(preselectTeleOp = "Blue Bot Teleop")
public class Park extends LinearOpMode {
  private Swerve drivebase;

  @Override
  public void runOpMode() throws InterruptedException {
    drivebase = new Swerve(this);
    drivebase.initGyro();

    waitForStart();
    drivebase.alignWheels(this::opModeIsActive);
    driveForTime(new ChassisSpeeds(0, -0.5, 0), 1);
  }

  private void driveForTime(ChassisSpeeds speeds, double time) {
    double startTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {
      drivebase.drive(speeds);
      if (Utils.getTimeSeconds() - startTime >= time) {
        return;
      }
    }
  }
}
