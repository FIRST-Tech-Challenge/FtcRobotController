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
  double motorSpeedAdjustment;
  int deltaWheelSpeed;

  public void initRobot(LinearOpMode opMode) {
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
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

      int referenceSpeed;
      int frWheelSpeed;
      int brWheelSpeed;
      int blWheelSpeed;
      motorSpeedAdjustment = 0.0;
      telemetry.addData("MotorFl current position: ", driveBase.motorFL.getCurrentPosition());
      telemetry.addData("MotorFR current position: ", driveBase.motorFR.getCurrentPosition());

      double angle = 0.5;
      while (true) {
        referenceSpeed = driveBase.motorFL.getCurrentPosition();
        frWheelSpeed = driveBase.motorFR.getCurrentPosition();
        brWheelSpeed = driveBase.motorBR.getCurrentPosition();
        blWheelSpeed = driveBase.motorBL.getCurrentPosition();

        double delta_sum = driveBase.set_wheels(angle, 0.49, 0.49, angle);

        if (delta_sum == 0.0) {
          driveBase.motorFL.setPower(0.5);

          deltaWheelSpeed = frWheelSpeed - referenceSpeed;
          if (deltaWheelSpeed > 0) {
            motorSpeedAdjustment = -0.2;
          } else if (deltaWheelSpeed < 0) {
            motorSpeedAdjustment = 0.2;
          }
          driveBase.motorFR.setPower(0.55 + motorSpeedAdjustment);

          deltaWheelSpeed = brWheelSpeed - referenceSpeed;
          if (deltaWheelSpeed > 0) {
            motorSpeedAdjustment = -0.2;
          } else if (deltaWheelSpeed < 0) {
            motorSpeedAdjustment = 0.2;
          }
          driveBase.motorBR.setPower(0.55 + motorSpeedAdjustment);

          deltaWheelSpeed = blWheelSpeed - referenceSpeed;
          if (deltaWheelSpeed > 0) {
            motorSpeedAdjustment = -0.2;
          } else if (deltaWheelSpeed < 0) {
            motorSpeedAdjustment = 0.2;
          }
          driveBase.motorBL.setPower(0.5 + motorSpeedAdjustment);
        }



        telem();
      }

//      driveBase.alignWheels();
//      driveBase.stopServo();
//      //checkMotorDirAndServoVolt();
//// drive to bar
//      // driveBase.driveDist(1.3, .5, driveAngle);// alignWheels is called within driveDist
//// raise arm
//// drive forward
//// lower to clip
//// back up
//// turn to yellow or blue samples
//      sleep(2500);
//      for(int i = 0;i<10;i++) {
//        telem();
//        driveBase.set_Servo_Angle(driveBase.servoInputFR, driveBase.servoFR, i*0.1);
//        sleep(1000);
//      }
//      driveBase.stopServo();
    }// end runOpMode

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
//    telemetry.addData("slide current: ", mek.slide.getCurrentPosition());
//    telemetry.addData("slide goal: ", mek.slide.getTargetPosition());
//    telemetry.addData("pivot current: ", mek.pivot.getCurrentPosition());
//    telemetry.addData("pivot goal: ", mek.pivot.getTargetPosition());
//    telemetry.addLine(". . . . . . . . . . . . . . . .");
//    telemetry.addData("odometry Status",driveBase.odo.getDeviceStatus());
//    telemetry.addData("odometry x: ",driveBase.odo.getPosX());
//    telemetry.addData("odometry y: ",driveBase.odo.getPosY());
//    telemetry.addData("odometry yaw: ",driveBase.odo.getHeading());
//    telemetry.addLine(". . . . . . . . . . . . . . . .");
    telemetry.addData("ServoFR voltage: ", driveBase.servoInputFR.getVoltage());
    telemetry.addData("servoFL voltage: ", driveBase.servoInputFL.getVoltage());
    telemetry.addData("ServoBR voltage: ", driveBase.servoInputBR.getVoltage());
    telemetry.addData("servoBL voltage: ", driveBase.servoInputBL.getVoltage());
    telemetry.addData("ServoFR center: ", driveBase.servoInputFR.getMaxVoltage() / 2.0);
    telemetry.addData("servoFL center: ", driveBase.servoInputFL.getMaxVoltage() / 2.0);
    telemetry.addData("ServoBR center: ", driveBase.servoInputBR.getMaxVoltage() / 2.0);
    telemetry.addData("servoBL center: ", driveBase.servoInputBL.getMaxVoltage() / 2 / 0);
    telemetry.addData("ServoFR max voltage: ", driveBase.servoInputFR.getMaxVoltage());
    telemetry.addData("servoFL max voltage: ", driveBase.servoInputFL.getMaxVoltage());
    telemetry.addData("ServoBR max voltage: ", driveBase.servoInputBR.getMaxVoltage());
    telemetry.addData("servoBL max voltage: ", driveBase.servoInputBL.getMaxVoltage());
    telemetry.addLine("-------------------------------");
    telemetry.addData("MotorFl current position: ", driveBase.motorFL.getCurrentPosition());
    telemetry.addData("MotorFR current position: ", driveBase.motorFR.getCurrentPosition());
    telemetry.addData("Motor speed adjustment: ",motorSpeedAdjustment);
    telemetry.addData("Delta wheel speed: ",deltaWheelSpeed);

    telemetry.update();
  }
}
