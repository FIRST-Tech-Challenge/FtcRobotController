// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class clawSlideImport extends LinearOpMode {

  DcMotor slide, pivot, hang;
  // DigitalChannel slideLimit;

  Servo intakeL, intakeR;
  int limitSlide, limitPivot;

  @Override
  public void runOpMode() throws InterruptedException {

    initRobot();

    waitForStart();

    while (opModeIsActive()) {

      if (gamepad2.a) {
        intakeL.setPosition(1);
        intakeR.setPosition(1);
      } else if (gamepad2.b) {
        intakeL.setPosition(0);
        intakeR.setPosition(0);
      }

      setSlide(-gamepad2.right_stick_y);
      setPivot(-gamepad2.left_stick_y);
      // telemetry.addData("SL", slideLimit.getState());
      telemetry.update();
    }
  }

  public void initRobot() {
    slide = hardwareMap.get(DcMotor.class, "slide");
    pivot = hardwareMap.get(DcMotor.class, "pivot");
    // slideLimit = hardwareMap.get(DigitalChannel.class, "slideLimit");

    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    slide.setPower(.01);
    pivot.setPower(.01);

    slide.setDirection(DcMotorSimple.Direction.FORWARD);
    pivot.setDirection(DcMotorSimple.Direction.REVERSE);

    limitSlide = 4750;
    limitPivot = 2500;

    // servos

    intakeL = hardwareMap.get(Servo.class, "intake");
    intakeR = hardwareMap.get(Servo.class, "wrist");

    intakeL.setDirection(Servo.Direction.REVERSE);
    intakeR.setDirection(Servo.Direction.REVERSE);

    hang = hardwareMap.get(DcMotor.class, "hang");
    hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    hang.setDirection(DcMotorSimple.Direction.FORWARD);
    hang.setPower(.01);
  }

  public void setSlide(double x) {
    telemetry.addData("set slide x 1", x);
    if (slide.getCurrentPosition() >= limitSlide && x > 0) {
      x = 0;
    } else if (slide.getCurrentPosition() <= -limitSlide && x < 0) {
      x = 0;
    }
    slide.setPower(x);
    hang.setPower(x);
    telemetry.addData("set slide x 2", x);
  }

  public void setPivot(double x) {
    telemetry.addData("set pivot x 1", x);
    if (pivot.getCurrentPosition() >= limitPivot && x > 0) {
      x = 0;
    } else if (pivot.getCurrentPosition() <= -limitPivot && x < 0) {
      x = 0;
    }
    pivot.setPower(x);
    telemetry.addData("set pivot x 2", x);
  }
}
