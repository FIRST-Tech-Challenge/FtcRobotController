// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;

@Autonomous(name = "Comp Bot Auto")
public class CompBotAuto extends LinearOpMode {

  Mekanism mek;

  AutoSwerve driveBase;

  GoBildaPinpointDriver odometry;

  public void initRobot(LinearOpMode opMode){
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.setOffsets(110, 30);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);
    odometry.resetHeading(Rotation2d.fromDegrees(120));

    mek = new Mekanism(opMode);
    driveBase = new AutoSwerve(this,odometry);
  }
  // ... Main ... runOpMode
  @Override
  public void runOpMode() throws InterruptedException {
    initRobot(this);
    // zero degrees is unstable do not use 360 or 0 degrees
    // use 180 and reverse motors
    int driveAngle = 180;
    waitForStart();
// initialize arm
    mek.homeArm();

// set direction
    if (opModeIsActive()) {
      PushPush();
      //driveBase.stopServo();
      // driveBase.alignWheels(driveAngle);
// drive to bar
      // driveBase.driveDist(1.3, .5, driveAngle);// alignWheels is called within driveDist
// raise arm
// drive forward
// lower to clip
// back up
// turn to yellow or blue samples
      telem();
      sleep(10000);
    }// end runOpMode

  }

  public void PushPush(){
    while(opModeIsActive()) {
      driveBase.stopServo();
      telem();
    }
  }

  public void checkMotorDirAndServoVolt(){
    driveBase.setMotors(.5);
    telem();
    sleep(5000);
    driveBase.setMotors(-.5);
    telem();
    sleep(5000);
  }
  // this is debug information
  public void telem() {
    driveBase.odo.update();
    driveBase.odo.getPose();
    telemetry.addData("slide current: ", mek.slide.getCurrentPosition());
    telemetry.addData("slide goal: ", mek.slide.getTargetPosition());
    telemetry.addData("pivot current: ", mek.pivot.getCurrentPosition());
    telemetry.addData("pivot goal: ", mek.pivot.getTargetPosition());
    telemetry.addLine(". . . . . . . . . . . . . . . .");
    telemetry.addData("odometry Status",driveBase.odo.getDeviceStatus());
    telemetry.addData("odometry x: ",driveBase.odo.getPosX());
    telemetry.addData("odometry y: ",driveBase.odo.getPosY());
    telemetry.addData("odometry yaw: ",driveBase.odo.getHeading());
    telemetry.addLine(". . . . . . . . . . . . . . . .");
    telemetry.addData("ServoFR voltage: ",driveBase.servoInputFR.getVoltage());
    telemetry.addData("servoFL voltage: ",driveBase.servoInputFL.getVoltage());
    telemetry.addData("ServoBR voltage: ",driveBase.servoInputBR.getVoltage());
    telemetry.addData("servoBL voltage: ",driveBase.servoInputBL.getVoltage());
    telemetry.addData("ServoFR center: ",driveBase.servoInputFR.getMaxVoltage()/2.0);
    telemetry.addData("servoFL center: ",driveBase.servoInputFL.getMaxVoltage()/2.0);
    telemetry.addData("ServoBR center: ",driveBase.servoInputBR.getMaxVoltage()/2.0);
    telemetry.addData("servoBL center: ",driveBase.servoInputBL.getMaxVoltage()/2/0);

    telemetry.update();
  }
}
