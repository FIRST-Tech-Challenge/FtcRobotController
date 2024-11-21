// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Comp Bot Auto", preselectTeleOp = "Blue Bot Teleop")
public class CompBotAuto extends LinearOpMode {

  DcMotor slide, slide2, pivot;
  Servo intakeL, wrist;
  int limitSlide, limitPivot;

  double pubLength = 0;

  double encoderCountsPerInch = 100; // needs adjusting

  double encoderCountsPerDegree = 30;

  boolean test = false;

  DigitalChannel limitSwitch;

  @Override
  public void runOpMode() throws InterruptedException {

    initRobot();
    waitForStart();
    //initSlide();

    slide.setTargetPosition(4000); // change to edit drop spot
    pivot.setTargetPosition(175); // change to edit drop spot

    sleep(3000);

    intakeL.setPosition(0);

    sleep(1000);
    intakeL.setPosition(0.5);
    pivot.setTargetPosition(0);
    slide.setTargetPosition(0);
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

    slide.setDirection(DcMotorSimple.Direction.FORWARD);
    pivot.setDirection(DcMotorSimple.Direction.FORWARD);

    limitSlide = 4750;
    limitPivot = 3200;

    // servos
    intakeL = hardwareMap.get(Servo.class, "intake");
    wrist = hardwareMap.get(Servo.class, "wrist");

    intakeL.setDirection(Servo.Direction.REVERSE);
    wrist.setDirection(Servo.Direction.FORWARD);

    wrist.setPosition(0.7);

    // limit switch and brings pivot back

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
