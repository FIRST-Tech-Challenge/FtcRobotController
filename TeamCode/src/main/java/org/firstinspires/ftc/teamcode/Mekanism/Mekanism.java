// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.MathUtil;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.util.Units;

public class Mekanism {
  private final DcMotorEx pivot;
  private final DcMotorEx slide;
  // Limit switch is never used??? - Tada
  private final DigitalChannel limitSwitch;

  private final Servo intakeL;
  private final Servo wrist;

  private final double limitSlide;
  private final double limitPivot;

  private double pubLength = 0;

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

    limitSlide = 4200;
    limitPivot = 2750;

    limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "limit switch");

    // servos for intake
    intakeL = opMode.hardwareMap.get(Servo.class, "intake");
    wrist = opMode.hardwareMap.get(Servo.class, "wrist");

    intakeL.setDirection(Servo.Direction.REVERSE);
    wrist.setDirection(Servo.Direction.FORWARD);

    wrist.setPosition(0.7);
  }

  // to lift arm, input from game pad 2 straight in
  public void setSlide(double x) {
    if (slide.getCurrentPosition() >= limitSlide && x > 0) {
      x = 0;
    } else if (slide.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }

    // needs adjusting
    double encoderCountsPerDegree = 30;
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
                      90 - (pivot.getCurrentPosition() * encoderCountsPerDegree)));

    double encoderCountsPerInch = 100;
    pubLength =
        Math.cos(Math.toRadians(pivot.getCurrentPosition() / encoderCountsPerDegree))
            * (46 * encoderCountsPerInch);
    if (pubLength <= 2950) {
      pubLength = 2950;
    }
    if (x > 0 && slide.getCurrentPosition() > pubLength) {
      x = 0;
    }

    slide.setPower(x);
  }

  public void setPivot(double x, boolean raiseLimit) {
    if (pivot.getCurrentPosition() >= (raiseLimit ? limitPivot : limitPivot + 500) && x > 0) {
      x = 0;
    } else if (pivot.getCurrentPosition() <= 0 && x < 0) {
      x = 0;
    }

    // TODO: Ask Philip what the hell this does
    if (x > 1) x = .5;
    if (x < -1) x = -.5;
    // The above makes this if statement impossible to trigger??? - Tada
    if (slide.getCurrentPosition() > pubLength && x > 1) slide.setPower(-x * 2);
    pivot.setPower(x);
  }

  public void moveClaw(boolean intake, boolean outtake) {
    // TODO: Does intake and outtake need their names flipped?
    if (intake) {
      intakeL.setPosition(1);
    } else if (outtake) {
      intakeL.setPosition(0);
    } else {
      intakeL.setPosition(.5);
    }
  }

  public void moveWrist(boolean move) {
    // TODO: Ask Philip why he did this, can't this just be a !move check? - Tada
    if (wrist.getPosition() >= .1) {
      wrist.setPosition(0);
    } else {
      wrist.setPosition(.7);
    }
  }
}
