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

  private void center_wheel_steering(AnalogInput analogInput, Servo servo)
  {
    double pot_voltage = analogInput.getVoltage();
    double max_voltage = 3.326;
    double normalized_voltage = pot_voltage / max_voltage;
    double delta_to_reference = 0.5 - normalized_voltage;

    telemetry.addData("Norm:  ", normalized_voltage);
    telemetry.addData("Delta: ", delta_to_reference);
    telemetry.update();

    double tolerance = 0.01;

    if (delta_to_reference < (-1*tolerance))
    {
      servo.setPosition(0.55);
    }
    else if (delta_to_reference > tolerance)
    {
      servo.setPosition(0.45);
    }
    else
    {
      servo.setPosition(0.5);
    }
  }

  @Override
  public void runOpMode() throws InterruptedException {
    initRobot(this);

    waitForStart();

    while(true)
    {
      center_wheel_steering(driveBase.servoInputFL, driveBase.servoFL);
      center_wheel_steering(driveBase.servoInputFR, driveBase.servoFR);
      center_wheel_steering(driveBase.servoInputBL, driveBase.servoBL);
      center_wheel_steering(driveBase.servoInputBR, driveBase.servoBR);
    }

//    mek.homeArm();
//    //driveBase.wheelAngle = 180;;
//    if(opModeIsActive()) {
//      //double drvAng = driveBase.wheelAngle;
//      double drvAng = 180;
//      // alignWheels locks the wheels in transit
//      // In auto change angle as require
//      driveBase.alignWheels(drvAng);
//
//
//      //driveBase.driveDist(2.0, .5);// alignWheels is called within driveDist
//    }
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
