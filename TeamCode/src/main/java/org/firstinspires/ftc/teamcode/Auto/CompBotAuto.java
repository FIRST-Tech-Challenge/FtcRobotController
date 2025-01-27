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



  @Override
  public void runOpMode() throws InterruptedException {
    initRobot(this);

    waitForStart();
    mek.homeArm();
    driveBase.alginWheels();

    while(true)
    {
      driveBase.set_Servo_Angle(driveBase.servoInputFL, driveBase.servoFL, 0.4);
      driveBase.set_Servo_Angle(driveBase.servoInputFR, driveBase.servoFR, 0.4);
      driveBase.set_Servo_Angle(driveBase.servoInputBL, driveBase.servoBL, 0.4);
      driveBase.set_Servo_Angle(driveBase.servoInputBR, driveBase.servoBR, 0.4);
    }
  }// end runOpMode

  public void initRobot(LinearOpMode opMode){
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.setOffsets(110, 30);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);
    odometry.resetHeading(Rotation2d.fromDegrees(120));

    mek = new Mekanism(opMode);
    driveBase = new AutoSwerve(this,odometry);
  }

  public void telem() {
    telemetry.addData("slide current: ", mek.slide.getCurrentPosition());
    telemetry.addData("slide goal: ", mek.slide.getTargetPosition());
    telemetry.addData("pivot current: ", mek.pivot.getCurrentPosition());
    telemetry.addData("pivot goal: ", mek.pivot.getTargetPosition());
    telemetry.addLine(". . . . . . . . . . . . . . . .");
    telemetry.addData("odometry x: ",odometry.getPosX());
    telemetry.addData("odometry y: ",odometry.getPosY());
    telemetry.addData("odometry yaw: ",odometry.getHeading());
    telemetry.addLine(". . . . . . . . . . . . . . . .");
    /*
    telemetry.addData("ServoFR voltage: ",driveBase.servoInputFR.getVoltage());
    telemetry.addData("servoFL voltage: ",driveBase.servoInputFL.getVoltage());
    telemetry.addData("ServoBR voltage: ",driveBase.servoInputBR.getVoltage());
    telemetry.addData("servoBL voltage: ",driveBase.servoInputBL.getVoltage());
     */
    telemetry.update();
  }
}
