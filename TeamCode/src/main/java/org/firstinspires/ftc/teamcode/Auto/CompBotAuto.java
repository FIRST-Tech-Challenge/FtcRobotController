// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;

@Autonomous(name = "Comp Bot Auto")
public class CompBotAuto extends LinearOpMode {

  Mekanism mek;

  AutoSwerve driveBase;

  GoBildaPinpointDriver odometry;
  double motorSpeedAdjustment = 0.0;
  int deltaWheelSpeed;

  public void initRobot(LinearOpMode opMode) {
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.recalibrateIMU();
    odometry.resetPosAndIMU();
    odometry.setOffsets(110, 30);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);
    odometry.resetHeading(Rotation2d.fromDegrees(120));

    mek = new Mekanism(opMode);
    driveBase = new AutoSwerve(this, odometry);
  }

//  private void set_relative_speed(DcMotor motor) {
//    int wheel = motor.getCurrentPosition() -
//  }

  // ... Main ... runOpMode
  @Override
  public void runOpMode() throws InterruptedException {
    initRobot(this);
    waitForStart();
    mek.homeArm();

// set direction
    if (opModeIsActive()) {

      while (opModeIsActive()) {

        if (driveBase.set_wheels(0.231, 0.24, 0.24, 0.259) == 0.0) {
          //drive_Wheels(0.5);
          goToPos(0.4,0.0,0.5);
          //PushPush();
        }
      }

//      rotate(45,0.5);
//
//      sleep(30000);
//
//      mek.setPivot(1,false);
//      sleep(500);
//      mek.setPivot(0,false);
//      mek.setSlide(4250);
//      sleep(5000);
//      mek.runIntake(false,true);
//      sleep(1000);
//      mek.runIntake(false,false);
//      sleep(500);
//      mek.setPivot(-1,false);
//      sleep(500);
//      mek.setPivot(0,false);
//      sleep(250);
//      mek.setSlide(0);
//      sleep(2500);
      telem();
      odometry.update();
    }// end runOpMode
    drive_Wheels(0,0);
    driveBase.stopServo();
  }

  public void rotate(double angle,double pwr){
    double change_Yaw = odometry.getHeading().getDegrees() - angle;

    while(change_Yaw > 0.01 || change_Yaw < -0.01){
      telem();
      if(driveBase.set_wheels(0.0,0.5,0.5,0.5) == 0.0){
        if(change_Yaw>0)
          drive_Wheels(pwr,2);
        else
          drive_Wheels(-pwr,2);
      }
      change_Yaw = odometry.getHeading().getDegrees() - angle;
    }
    drive_Wheels(0,2);
  }

  public void goToPos(double xPos,double yPos,double pwr){
    double change_X = odometry.getPosX() + xPos;
    double change_Y = odometry.getPosY() - yPos;

    //change x pos
    while(change_X>0.01 || change_X<-0.01){
      if(xPos==0)
        break;
      change_X = odometry.getPosX() + xPos;  //get change
      telemetry.addData("X change: ",change_X);
      if(driveBase.set_wheels(0.5, 0.49, 0.49, 0.5)==0){
        if(change_X>0)
          drive_Wheels(pwr,0);
        else
          drive_Wheels(-pwr,0);
      }
      odometry.update();
      telem();
      if(!opModeIsActive())
        return;
    }
    drive_Wheels(0,0);

    //change y pos
    while(change_Y>0.01 || change_Y<-0.01){
      if(yPos==0)
        break;
      change_Y = odometry.getPosY() - yPos;  //get change
      telemetry.addData("Y change: ",change_Y);
      if(driveBase.set_wheels(0.231, 0.24, 0.24, 0.259)==0){
        if(change_Y>0)
          drive_Wheels(-pwr,1);
        else
          drive_Wheels(pwr,1);
      }
      odometry.update();
      telem();
      if(!opModeIsActive())
        return;
    }
    drive_Wheels(0,1);
  }

  public void drive_Wheels(double power,int Switch){

    //Switch is a switch between forward, strafe and rotate power for the motors
    //                              0   ,    1   and   2
    driveBase.motorFL.setPower(power);
    int referenceSpeed = driveBase.motorFL.getCurrentPosition();

    deltaWheelSpeed = driveBase.motorFR.getCurrentPosition() - referenceSpeed;
    if (deltaWheelSpeed > 0) {
      motorSpeedAdjustment = -0.2;
    } else if (deltaWheelSpeed < 0) {
      motorSpeedAdjustment = 0.2;
    }
    driveBase.motorFR.setPower(0.55 + motorSpeedAdjustment);

    deltaWheelSpeed = driveBase.motorBR.getCurrentPosition() - referenceSpeed;
    if (deltaWheelSpeed > 0) {
      motorSpeedAdjustment = -0.2;
    } else if (deltaWheelSpeed < 0) {
      motorSpeedAdjustment = 0.2;
    }
    driveBase.motorBR.setPower(0.55 + motorSpeedAdjustment);

    deltaWheelSpeed = driveBase.motorBL.getCurrentPosition() - referenceSpeed;
    if (deltaWheelSpeed > 0) {
      motorSpeedAdjustment = -0.2;
    } else if (deltaWheelSpeed < 0) {
      motorSpeedAdjustment = 0.2;
    }
    driveBase.motorBL.setPower(0.5 + motorSpeedAdjustment);
  }

  public void PushPush() {
    while (opModeIsActive()) {
      driveBase.stopServo();
      telem();
    }
  }

  public void checkMotorDirAndServoVolt() {
    driveBase.setMotors(.5);
    telem();
    sleep(5000);
    driveBase.setMotors(-.5);
    telem();
    sleep(5000);
    driveBase.setMotors(0);
    telem();
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
//    telemetry.addLine(". . . . . . . . . . . . . . . .");
//    telemetry.addData("ServoFR voltage: ", driveBase.servoInputFR.getVoltage());
//    telemetry.addData("servoFL voltage: ", driveBase.servoInputFL.getVoltage());
//    telemetry.addData("ServoBR voltage: ", driveBase.servoInputBR.getVoltage());
//    telemetry.addData("servoBL voltage: ", driveBase.servoInputBL.getVoltage());
//    telemetry.addData("ServoFR center: ", driveBase.servoInputFR.getMaxVoltage() / 2.0);
//    telemetry.addData("servoFL center: ", driveBase.servoInputFL.getMaxVoltage() / 2.0);
//    telemetry.addData("ServoBR center: ", driveBase.servoInputBR.getMaxVoltage() / 2.0);
//    telemetry.addData("servoBL center: ", driveBase.servoInputBL.getMaxVoltage() / 2 / 0);
//    telemetry.addData("ServoFR max voltage: ", driveBase.servoInputFR.getMaxVoltage());
//    telemetry.addData("servoFL max voltage: ", driveBase.servoInputFL.getMaxVoltage());
//    telemetry.addData("ServoBR max voltage: ", driveBase.servoInputBR.getMaxVoltage());
//    telemetry.addData("servoBL max voltage: ", driveBase.servoInputBL.getMaxVoltage());
//    telemetry.addLine("-------------------------------");
//    telemetry.addData("MotorFl current position: ", driveBase.motorFL.getCurrentPosition());
//    telemetry.addData("MotorFR current position: ", driveBase.motorFR.getCurrentPosition());
//    telemetry.addData("Motor speed adjustment: ",motorSpeedAdjustment);
//    telemetry.addData("Delta wheel speed: ",deltaWheelSpeed);

    telemetry.update();
  }
}
