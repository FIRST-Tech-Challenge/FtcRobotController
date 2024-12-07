// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.MathUtil;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.util.Units;

public class Mekanism {
  private final DcMotorEx pivot;
  private final DcMotorEx slide;
  private final DcMotorEx slide2;

  private final DigitalChannel limitSwitch;

  private final Servo intakeServo;
  private final Servo wrist;

  private final Servo ramp1;
  private final Servo ramp2;

  private final double limitSlide;
  private final double limitPivot;

  private final Telemetry telemetry;

  public Mekanism(LinearOpMode opMode) {
    // Init slaw, claw, and pivot
    pivot = (DcMotorEx) opMode.hardwareMap.dcMotor.get("pivot");
    slide = (DcMotorEx) opMode.hardwareMap.dcMotor.get("slide");

    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);

    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_USING_ENCODER);

    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    pivot.setDirection(FORWARD);
    slide.setDirection(FORWARD);

    pivot.setPower(0);
    slide.setPower(0);

    // slide2
    slide2 = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "slide 2");
    slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slide2.setDirection(DcMotorSimple.Direction.FORWARD);
    slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide2.setPower(0);

    limitSlide = 4200;
    limitPivot = 2750;

    limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "limit switch");

    // servos for intake
    intakeServo = opMode.hardwareMap.get(Servo.class, "intake");
    wrist = opMode.hardwareMap.get(Servo.class, "wrist");

    intakeServo.setDirection(Servo.Direction.REVERSE);
    wrist.setDirection(Servo.Direction.REVERSE);

    wrist.setPosition(0.65);

    // servos for clipper
    ramp1 = opMode.hardwareMap.get(Servo.class, "ramp 1");
    ramp2 = opMode.hardwareMap.get(Servo.class, "ramp 2");

    ramp1.setDirection(Servo.Direction.FORWARD);
    ramp2.setDirection(Servo.Direction.FORWARD);

    unclamp();

    ramp1.scaleRange(0, 1);
    ramp2.scaleRange(0, 1);

    telemetry = opMode.telemetry;
  }

  // to extend arm, input from game pad 2 straight in
  public void setSlide(double x) {
    if (slide.getCurrentPosition() >= limitSlide && x > 0) {
      x = 0;
    } else if (slide.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }

    telemetry.addData("slide current pos", slide.getCurrentPosition()); // needs adjusting
    // TODO: Tuning is very vibes based because this value is very wrong
    double encoderCountsPerDegree = 30;
    /*          // because 2 motors now so not needed
    if (x == 0)
      x =
          // kG math
          // kG is proportionally related to extension distance somehow???
          // Prolly weird friction with the linear slide - Tada
          MathUtil.interpolate(
                  .00125,
                  .005,
                  MathUtil.inverseInterpolate(0, limitSlide, slide.getCurrentPosition()))
              // kG needs to be scaled with the sin of angle
              * Math.sin(
                  Units.degreesToRadians(
                      90 - (pivot.getCurrentPosition() / encoderCountsPerDegree)));
     */

    // TODO: Tuning is very vibes based because this value is very wrong, fix it
    double encoderCountsPerInch = 85;
    // Prevent divide by 0
    double pubLength =
        Math.min(
            (29.5 * encoderCountsPerInch)
                / Math.max(
                    Math.cos(
                        Math.toRadians(90 - (pivot.getCurrentPosition() / encoderCountsPerDegree))),
                    1e-6), // Prevent divide by 0
            48 * encoderCountsPerInch); // Limit extension
    if (slide.getCurrentPosition() > pubLength) {
      x = -.5;
    }

    slide.setPower(x);
    slide2.setPower(x);
  }

  public void setPivot(double x, boolean raiseLimit) {
    if (pivot.getCurrentPosition() >= (raiseLimit ? limitPivot : limitPivot + 500) && x > 0) {
      x = 0;
    } else if (pivot.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }
    telemetry.addData("pivot current pos", pivot.getCurrentPosition());

    x *= .5;
    pivot.setPower(x);
  }

  public void homeArm() {
    while (limitSwitch.getState()) {
      pivot.setPower(-.75);
      slide.setPower(-0.5);
    }

    pivot.setPower(.00);
    slide.setPower(.00);
    try {
      Thread.sleep(250);
    } catch (final InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
    pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    pivot.setPower(.00);
  }

  public void runIntake(boolean intake, boolean outtake) {
    if (outtake) {
      intakeServo.setPosition(1);
    } else if (intake) {
      intakeServo.setPosition(0);
    } else {
      intakeServo.setPosition(.5);
    }
  }

  public void intake() {
    intakeServo.setPosition(1);
  }

  public void outtake() {}

  public void clamp() {
    ramp1.setPosition(0);
    ramp2.setPosition(.15);
  }

  public void unclamp() {
    ramp1.setPosition(.15);
    ramp2.setPosition(0);
  }

  public void autoClip() {
    telemetry.addData("pivot current pos", pivot.getCurrentPosition());
    telemetry.addData("slide current pos", slide.getCurrentPosition());
    if (slide.getCurrentPosition() > 150) {
      unclamp();
      slide.setPower(-1);
      wrist.setPosition(0.1);
    } else {
      clamp();
      slide.setPower(0);
      if (pivot.getCurrentPosition() < 2200) {
        wrist.setPosition(0.1);
        pivot.setPower(1);
      } else {
        wrist.setPosition(0);
        slide.setPower(-1);
        if (pivot.getCurrentPosition() < 2500) {
          pivot.setPower(1);
        } else {
          pivot.setPower(0);
        }
      }
    }
    wristMoved = true;
  }

  private boolean wristMoved = false;

  public void toggleWrist() {
    if (!wristMoved) {
      wrist.setPosition(0.1);
      wristMoved = true;
    } else {
      wrist.setPosition(0.65);
      wristMoved = false;
    }
  }

  public void topSpecimenRung() {
    // setPivot();
    // setSlide();
  }
}
