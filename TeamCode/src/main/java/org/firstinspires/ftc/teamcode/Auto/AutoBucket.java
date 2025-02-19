// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;


@Autonomous(name = "Auto Top Bucket")
public class AutoBucket extends LinearOpMode {
  private AutoSwerve drivebase;
  private GoBildaPinpointDriver odo;
  public Telemetry telemetry;

  @Override
  public void runOpMode() throws InterruptedException {
    initOdo();
    drivebase = new AutoSwerve(this,odo);

    waitForStart();
    // raise arm to top bucket drop and then move out to second one
  }

  public void initOdo(){
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.recalibrateIMU();
    odo.resetPosAndIMU();
    odo.setOffsets(110, 30);
    odo.setEncoderResolution(goBILDA_4_BAR_POD);
    odo.setEncoderDirections(FORWARD, FORWARD);
    odo.resetHeading(Rotation2d.fromDegrees(120));
  }

  /**
   *  targetPos Pose2d target position relative to the field. 0,0 is where the robot first started
   *  timeLimit Time to take to move the specified distance
   * needs to be entirely rewritten
   */
  /*
  public void driveWithOdo(Pose2d targetPos, double timeLimit) {

    ElapsedTime timer = new ElapsedTime();
    timer.reset();

    Pose2d currentPos;

    double
        integralX = 0,
        integralY = 0,
        integralHeading = 0,
        previousErrorX = 0,
        previousErrorY = 0,
        previousErrorHeading = 0,
        lastTime = 0;

    while (timer.seconds() < timeLimit && opModeIsActive()) {

      // 1. Get the current position of the robot
      currentPos = drivebase.getPose();

      // 2. Calculate the error in position
      double errorX = targetPos.getX() - currentPos.getX();
      double errorY = targetPos.getY() - currentPos.getY();
      double errorHeading = targetPos.getRotation().getRadians() - currentPos.getRotation().getRadians();

      // 3. Calculate the time since the last loop through
      double currentTime = timer.startTime();
      double deltaTime = currentTime - lastTime;

      // 4. Calculate the PID outputs
      double pidOutputX = calculatePID(errorX, deltaTime, 0.05, 0.0, 0.0, integralX, previousErrorX);
      double pidOutputY = calculatePID(errorY, deltaTime, 0.05, 0.0, 0.0, integralY, previousErrorY);
      double pidOutputHeading = calculatePID(errorHeading, deltaTime, 0.05, 0.0, 0.0, integralHeading, previousErrorHeading);

      // 5. Update the integral and previous error
      integralX += errorX * deltaTime;
      integralY += errorY * deltaTime;
      integralHeading += errorHeading * deltaTime;
      previousErrorX = errorX;
      previousErrorY = errorY;
      previousErrorHeading = errorHeading;

      // 6. Create speeds
      ChassisSpeeds speeds = new ChassisSpeeds(pidOutputX, pidOutputY, pidOutputHeading);

      // 7. Update speeds for the robot
      drivebase.drive(speeds, deltaTime);
      /*
      // 8. Telemetry - Telemetry causes a null exception for some reason
      telemetry.addData("Current X",0);
      telemetry.addData("Current Y", 0);
      telemetry.addData("Current Heading", 0);
      telemetry.addData("Time Remaining", 0);
      telemetry.update();

      // 9. Check if its close enough to the target
      if (Math.abs(errorX) < 0.5 && Math.abs(errorY) < 0.5 && Math.abs(errorHeading) < Math.toRadians(5)) {
        break;
      }

      if (timer.seconds() > timeLimit) {
        break;
      }
    }

    // Stops the robot
    drivebase.drive(new ChassisSpeeds(0, 0, 0), 0);
  }
  */


  private double calculatePID(double error, double deltaTime, double kP, double kI, double kD, double integral, double previousError) {
    double derivative = (error - previousError) / deltaTime;
    return kP * error + kI * integral + kD * derivative;
  }
}
