// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Auto.AutoSwerve;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  double slideSpeed = 80;
  GoBildaPinpointDriver odometry;

  AutoSwerve driveBase;
  boolean d_pad = false;
  //to save
  boolean lastPressed = false;

  public double frOffset = -0.125;
  public double brOffset = -0.125;
  public double blOffset = -0.25;
  public double flOffset = -0.25;
  public double frRotationOffset = -0.125;
  public double brRotationOffset = -0.125;
  public double blRotationOffset = -0.25;
  public double flRotationOffset = -0.25;

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

    steer_wheels(previous_steer_direction);

    while (opModeIsActive()) {



      double strafe_joystick = gamepad1.left_stick_x;
      double drive_joystick = -1 * gamepad1.left_stick_y;
      double rotate_joystick = gamepad1.right_stick_x;
      double robot_direction = odometry.getHeading().getRadians() - Math.PI/2;

      double odo_x = Math.cos(robot_direction);
      double odo_y = Math.sin(robot_direction);

      drive_joystick += odo_y + 1;
      drive_joystick *= -1;
      strafe_joystick += odo_x;
      
      drive_joystick = Math.sqrt(Math.pow(strafe_joystick, 2.0) + Math.pow(drive_joystick, 2.0));
      if (gamepad1.left_stick_y < 0) {
        drive_joystick *= -1.0;
      }

      double steer_direction = (strafe_joystick + 1.0) / 2.0;

      // Adjust wheel drive speed if needed
      if (drive_joystick != previous_driving_speed) {
        previous_driving_speed = drive_joystick;
        drive_wheels(drive_joystick);
      }
      
      // Adjust steering direction if needed
      if (steer_direction != previous_steer_direction) {
        previous_steer_direction = steer_direction;
        steer_wheels(steer_direction);
      }
      telemetry.addData("telemtry heading: ",robot_direction);
      telemetry.addData("steer direction: ",steer_direction);
      telemetry.update();
    }
  }

  private void steer_wheels(double x_direction) {
    driveBase.servoBL.setPosition(x_direction);
    driveBase.servoBR.setPosition(x_direction);
    driveBase.servoFL.setPosition(x_direction);
    driveBase.servoFR.setPosition(x_direction);
  }

  private void wheels(double input){
    double output = input;

    drive_wheels(output);
  }

  private void drive_wheels(double drive_speed) {
    driveBase.motorBL.setPower(drive_speed);
    driveBase.motorBR.setPower(drive_speed);
    driveBase.motorFL.setPower(drive_speed);
    driveBase.motorFR.setPower(drive_speed);
  }

  public double fieldOrientedHeading(){
    double heading = odometry.getHeading().getDegrees() / 360;
    return heading;
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

  public static double getAngle(double x, double y){
    double ret = 90 - Math.toDegrees(Math.atan2(x,y));
    if(ret<0)
      ret+=360;
    return ret;
  }
}
