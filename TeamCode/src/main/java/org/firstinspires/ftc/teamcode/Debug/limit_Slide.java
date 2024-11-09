// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class limit_Slide extends LinearOpMode {

  DcMotor slide, pivot;
  Servo intakeL, wrist;
  int limitSlide, limitPivot;

  double pubLength = 0;

  double encoderCountsPerInch = 100; // needs adjusting

  double encoderCountsPerDegree = 30;

  private DigitalChannel limitSwitch;

  @Override
  public void runOpMode() throws InterruptedException {

    initRobot();

    initSlide();

    waitForStart();

    while (opModeIsActive()) {
      double x = 0;
      if (gamepad2.left_trigger == 0) x = .5;
      double y = 0;
      if (gamepad2.right_trigger == 0) y = 0.5;

      if (x != 0.5) {
        intakeL.setPosition(x);
      } else if (y != 0.5) {
        intakeL.setPosition(y);
      }

      setSlide(-gamepad2.right_stick_y);
      setPivot(-gamepad2.left_stick_y);
      slideLimit();
      telemetry.addData("Limit switch: ", limitSwitch.getState());
      telemetry.addData("current pos: ", pivot.getCurrentPosition());
      telemetry.update();
    }
  }

  public void initRobot() {
    slide = hardwareMap.get(DcMotor.class, "slide");
    pivot = hardwareMap.get(DcMotor.class, "pivot");

    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    slide.setPower(.00);
    pivot.setPower(.00);

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
    double i = -0.75;
    while (limitSwitch.getState()) {
      pivot.setPower(i);
      telemetry.addData("pivot", pivot.getCurrentPosition());
      telemetry.addLine("initializing slide");
      telemetry.update();
    }
    pivot.setPower(.00);
    sleep(250);
    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    pivot.setPower(.00);
  }

  public void setSlide(double x) {
    if (slide.getCurrentPosition() >= limitSlide && x > 0) {
      x = 0;
    } else if (slide.getCurrentPosition() <= -limitSlide && x < 0) {
      x = 0;
    }
    if (x > 0 && slide.getCurrentPosition() > pubLength) {
      x = 0;
    }
    // TODO: add forced move for slide based on pivot
    slide.setPower(x);
  }

  public void slideLimit() {
    pubLength =
        Math.cos(Math.toRadians(pivot.getCurrentPosition() / encoderCountsPerDegree))
            * (46 * encoderCountsPerInch);
  }

  public void setPivot(double x) {
    if (pivot.getCurrentPosition() >= limitPivot && x > 0) {
      x = 0;
    } else if (pivot.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }

    if (x > 1) x = .5;
    if (x < -1) x = -.5;
    if (slide.getCurrentPosition() > pubLength && x > 1) slide.setPower(-x * 2);
    pivot.setPower(x);
  }
}
