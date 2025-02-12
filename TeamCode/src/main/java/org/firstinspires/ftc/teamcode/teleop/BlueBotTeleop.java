// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Auto.AutoSwerve;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  double slideSpeed = 80;
  GoBildaPinpointDriver odometry;

  AutoSwerve driveBase;
  boolean d_pad = false;
  boolean lastPressed = false;

  public double frOffset = -0.125;
  public double brOffset = -0.125;
  public double blOffset = -0.25;
  public double flOffset = -0.25;
  public double frRotationOffset = -0.125;
  public double brRotationOffset = -0.125;
  public double blRotationOffset = -0.25;
  public double flRotationOffset = -0.25;

  public final double change_In_Offset = .025;

  double normalize_angle(double angle) {
      angle += (Math.PI / 2.0);
      
      // If the robot is turning right, the theta angle can become negative.
      // The angle should remain within the range [0,2*pi).
      while (angle < 0.0) {
        angle += 2 * Math.PI;
      }

      return angle;
  }

  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? not anymore but you need to init the wrist or press b to get it to go to the right position
    Mekanism mek = new Mekanism(this);

    double slideSpeed = 100;
    boolean bPressed = false;

    Init();
    waitForStart();
    mek.initWrist();
    mek.homeArm();

    double previous_steer_direction = 0.5;
    double previous_driving_speed = 0.0;

    odometry.resetHeading();
    odometry.resetPosAndIMU();
    steer_wheels(previous_steer_direction);
    
    while (opModeIsActive()) {
      
      // Read raw input from left joystick
      double left_joy_x = gamepad1.left_stick_x;
      double left_joy_y = gamepad1.left_stick_y;

      // Reverse joystick y-axis to ensure 1 is up and -1 is down
      left_joy_y *= -1;

      telemetry.addLine("Left joy x: " + left_joy_x);
      telemetry.addLine("Left joy y: " + left_joy_y);

      // Compute theta of the left joystick vector
      double joy_theta = Math.atan2(left_joy_y, left_joy_x);

      // joy_theta = normalize_angle(joy_theta, false);

      telemetry.addLine("Joy theta: " + joy_theta);

      // Compute vector magnitude, this will be the speed of the robot
      double joy_magnitude = Math.sqrt(Math.pow(left_joy_x, 2.0) + Math.pow(left_joy_y, 2.0));

      telemetry.addLine("Joy mag: " + joy_magnitude);

      // Read raw value from odometer (in radians)
      double robot_theta = odometry.getHeading().getRadians();
      odometry.update();

      // Normalize odometry reading to align with standard polar coordinates
      robot_theta = normalize_angle(robot_theta);
      telemetry.addLine("Robot theta: " + robot_theta);

      // Compute steering angle relative to field-centric movements
      double steering_angle = joy_theta - robot_theta;
      if (steering_angle < -1 * Math.PI) {
        steering_angle = (steering_angle + Math.PI);
      }
      steering_angle = steering_angle % (2.0 * Math.PI);

      telemetry.addLine("Steering to: " + steering_angle);

      if (steering_angle > Math.PI) {
        steering_angle -= Math.PI;
        // Set flag to reverse movement
      }

      // Normalize steering to 0 to 1 range
      steering_angle /= Math.PI;

      telemetry.addLine("Normalized steering: " + steering_angle);

      telemetry.update();

      // steer_wheels(steering_angle);
    }
  }

  public void Init() {
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.recalibrateIMU();
    odometry.resetPosAndIMU();
    odometry.setOffsets(110, 30);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);
    odometry.resetHeading(Rotation2d.fromDegrees(120));
    driveBase = new AutoSwerve(this, odometry);
  }

  private void steer_wheels(double x_direction) {
    driveBase.servoBL.setPosition(x_direction);
    driveBase.servoBR.setPosition(x_direction);
    driveBase.servoFL.setPosition(x_direction);
    driveBase.servoFR.setPosition(x_direction);
  }

  private void drive_wheels(double drive_speed) {
    driveBase.motorBL.setPower(drive_speed);
    driveBase.motorBR.setPower(drive_speed);
    driveBase.motorFL.setPower(drive_speed);
    driveBase.motorFR.setPower(drive_speed);
  }

  public double fieldOrientedHeading() {
    double heading = odometry.getHeading().getDegrees() / 360;
    return heading;
  }

  public static double getAngle(double x, double y) {
    double ret = 90 - Math.toDegrees(Math.atan2(x, y));
    if (ret < 0)
      ret += 360;
    return ret;
  }
}
