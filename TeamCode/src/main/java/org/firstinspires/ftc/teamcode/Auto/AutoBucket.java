// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.Swerve.TheBestSwerve;


@Autonomous(name = "Auto Top Bucket")
public class AutoBucket extends LinearOpMode {
  private AutoSwerve driveBase;
  private GoBildaPinpointDriver odo;
  private Mekanism mek;
  TheBestSwerve amazingSwerve;

  @Override
  public void runOpMode() throws InterruptedException {
    initOdo();
    driveBase = new AutoSwerve(this, odo);
    mek = new Mekanism(this);
    amazingSwerve = new TheBestSwerve(this, odo, driveBase);

    waitForStart();
    mek.arm.homeArm();
    mek.grabber.initWrist();
    mek.grabber.setWrist(-1.0);
    mek.grabber.setGrabber(0, 0);
    mek.update();

    // raise arm to top bucket
    //topBucket();

    //Move robot to pick up second block
    //moveRobot(1.0, 1.0, 0.0);
    amazingSwerve.swerveTheThing(-1,-1,0);
    sleepWithAmazingSwerve(500);
    sleep(500);
  }

  public void topBucket(){
    mek.arm.setSlide(4100);
    sleepWithMekUpdate(2500);
    mek.arm.setPivot(15);
    sleepWithMekUpdate(1500);
    mek.grabber.setGrabber(1, .75);
    sleepWithMekUpdate(1000);
    mek.grabber.setGrabber(0, 0);
    mek.arm.setPivot(0);
    sleepWithMekUpdate(500);
    mek.arm.setSlide(0);
    sleepWithMekUpdate(2500);
  }

  public void sleepWithMekUpdate(int timeInMS) {
    double currentTime = Utils.getTimeMiliSeconds();
    double endTime = Utils.getTimeMiliSeconds() + timeInMS;
    while (currentTime < endTime && opModeIsActive()) {
      mek.update();
      currentTime = Utils.getTimeMiliSeconds();
      try {
        telemetry.addData("Slide pos: ", mek.arm.slide.getCurrentPosition());
        telemetry.addData("Pivot pos: ", mek.arm.pivot.getCurrentPosition());
        telemetry.addLine("grabber1 power: "+mek.grabber.intake1.getPosition());
        telemetry.addLine("grabber2 power: "+mek.grabber.intake2.getPosition());
        telemetry.addLine("pivot target pos: "+mek.arm.pivot.getTargetPosition());
        telemetry.addLine("pivot current pos: "+mek.arm.pivot.getCurrentPosition());
        telemetry.addLine("pivot power: "+mek.arm.pivot.getPower());
      } catch (Exception e) {
        telemetry.addLine("error");
      }
      telemetry.update();
    }
  }

  public void sleepWithAmazingSwerve(double timeInMS){
    double currentTime = Utils.getTimeMiliSeconds();
    double endTime = Utils.getTimeMiliSeconds() + timeInMS;
    while (currentTime < endTime && opModeIsActive()) {
      odo.update();
      currentTime = Utils.getTimeMiliSeconds();
    }
  }

  public void initOdo() {
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odo.recalibrateIMU();
    odo.resetPosAndIMU();
    odo.setOffsets(110, 30);
    odo.setEncoderResolution(goBILDA_4_BAR_POD);
    odo.setEncoderDirections(FORWARD, FORWARD);
    odo.resetHeading(Rotation2d.fromDegrees(120));
  }

  public void moveRobot(double strafe_x, double strafe_y, double steer_amt) {
    double odo_multiplier = 5;
    double change_x = strafe_x + odo.getPosX() * odo_multiplier;
    double change_y = strafe_y - odo.getPosY() * odo_multiplier;
    boolean y_was_0 = false;

    while (!(change_x > 0.05) || !(change_x < 0.05)   ||   !(change_y > 0.05) || !(change_y < 0.05)) {

      //reduce input to swerve
      double limit = 0.45;
      if(change_x > limit)
        change_x = limit;
      if(change_x < -limit)
        change_x = -limit;
      if(change_y > limit)
        change_y = limit;
      if(change_y < -limit)
        change_y = -limit;
      amazingSwerve.swerveTheThing(-change_x, change_y, 0.0);

      //reset input and print input
      change_x = strafe_x - odo.getPosX() * odo_multiplier;
      change_y = strafe_y - odo.getPosY() * odo_multiplier;
      change_y *= -1;
      outputPosition();
      telemetry.addLine("change x: " + change_x);
      telemetry.addLine("change y: " + -change_y);
      telemetry.update();
      odo.update();
    }
  }

  public void outputPosition(){
    telemetry.addLine("x pos: " + odo.getPosX() * 3);
    telemetry.addLine("y pos: " + odo.getPosY() * 3);
    telemetry.addLine("heading: " + odo.getHeading().getDegrees());
  }

  /**
   * targetPos Pose2d target position relative to the field. 0,0 is where the robot first started
   * timeLimit Time to take to move the specified distance
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
