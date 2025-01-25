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
    //driveBase.wheelAngle = 180;;
    if(opModeIsActive()) {
      //double drvAng = driveBase.wheelAngle;
      double drvAng = 180;
      // alignWheels locks the wheels in transit
      // In auto change angle as required
      driveBase.servoFL.setPosition(.5);
      driveBase.servoFR.setPosition(.5);
      driveBase.servoBL.setPosition(.5);
      driveBase.servoBR.setPosition(.5);
      //driveBase.alignWheels(drvAng);// sets wheel position
      driveBase.forward(.5);
      sleep(1000);
      driveBase.stopMotor();
      telemetry.addData("FL",driveBase.servoInputFL.getVoltage());
      telemetry.addData("FR",driveBase.servoInputFR.getVoltage());
      telemetry.addData("BL",driveBase.servoInputBL.getVoltage());
      telemetry.addData("BR",driveBase.servoInputBR.getVoltage());
      sleep(5000);

      //driveBase.driveDist(2.0, .5);// alignWheels is called within driveDist
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
