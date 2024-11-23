// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;

@Autonomous(name = "Comp Bot Auto", preselectTeleOp = "Blue Bot Teleop")
public class CompBotAuto extends LinearOpMode {

  DcMotor slide, slide2, pivot;
  Servo intake, wrist;
  int limitSlide, limitPivot;

  double encoderCountsPerInch = 100; // needs adjusting

  double encoderCountsPerDegree = 30;

  DigitalChannel limitSwitch;

  @Override
  public void runOpMode() throws InterruptedException {
    initRobot();
    waitForStart();
    // initSlide();
    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    slide.setTargetPosition(4100);
    slide2.setTargetPosition(4100);
    pivot.setTargetPosition(-1100);   //change
    wrist.setPosition(.35);
    //telem();

    sleep(4000);

    intake.setPosition(0);

    sleep(1000);
    intake.setPosition(0.5);
    pivot.setTargetPosition(0);
    slide.setTargetPosition(0);
    slide2.setTargetPosition(0);
    //telem();
    sleep(4500);
  }

  public void telem(){
    telemetry.addData("slide current: ",slide.getCurrentPosition());
    telemetry.addData("slide goal: ",slide.getTargetPosition());
    telemetry.addData("pivot current: ",pivot.getCurrentPosition());
    telemetry.addData("pivot goal: ",pivot.getTargetPosition());
    telemetry.addData("wrist current: ",wrist.getPosition());
    telemetry.update();
  }

  public void initRobot() {
    slide = hardwareMap.get(DcMotor.class, "slide");
    slide2 = hardwareMap.get(DcMotor.class, "slide 2");
    pivot = hardwareMap.get(DcMotor.class, "pivot");

    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    slide.setTargetPosition(0);
    slide2.setTargetPosition(0);
    pivot.setTargetPosition(0);

    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    slide.setPower(.90);
    slide2.setPower(.90);
    pivot.setPower(.5);

    slide.setDirection(DcMotorSimple.Direction.FORWARD);
    slide2.setDirection(DcMotorSimple.Direction.FORWARD);
    pivot.setDirection(DcMotorSimple.Direction.FORWARD);

    limitSlide = 4400;
    limitPivot = 3200;

    // servos
    intake = hardwareMap.get(Servo.class, "intake");
    wrist = hardwareMap.get(Servo.class, "wrist");

    intake.setDirection(Servo.Direction.REVERSE);
    wrist.setDirection(Servo.Direction.FORWARD);

    // limit switch and brings pivot back                 // currently not used
    limitSwitch = hardwareMap.get(DigitalChannel.class, "limit switch");
  }

  public void initSlide() {
    double i = 0.75;
    while (limitSwitch.getState()) {
      pivot.setPower(i);
      slide.setPower(-.01);
    }

    pivot.setPower(.00);
    slide.setPower(.00);
    sleep(250);

    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    pivot.setPower(.00);
  }
}
