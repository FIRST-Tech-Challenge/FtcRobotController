// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

  LinearOpMode myOp;

  ElapsedTime pivotTimer = new ElapsedTime();

  private final int limitSlide, limitPivot;
  private final double countsPerDegree, countsPerInch;
  private final Telemetry telemetry;

  public final DcMotorEx pivot;
  public final DcMotorEx slide, slide2;

  public final DigitalChannel limitSwitch;

  private int pivotTarget, slideTarget;


  public Arm(LinearOpMode opMode, int limitSlide, int limitPivot, double countsPerDegree, double countsPerInch) {

    // Init slaw, claw, and pivot
    pivot = (DcMotorEx) opMode.hardwareMap.dcMotor.get("pivot");
    slide = (DcMotorEx) opMode.hardwareMap.dcMotor.get("slide");
    slide2 = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "slide 2");

    // Sets rotation direction for the motors
    pivot.setDirection(DcMotorSimple.Direction.REVERSE);
    slide.setDirection(DcMotorSimple.Direction.FORWARD);
    slide2.setDirection(DcMotorSimple.Direction.FORWARD);

    // Reset all encoders
    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);
    slide2.setMode(STOP_AND_RESET_ENCODER);

    // Make SURE target position is set
    pivot.setTargetPosition(0);
    slide.setTargetPosition(0);
    slide2.setTargetPosition(0);

    // Set all mek motors RUN_USING_ENCODER mode
    pivot.setMode(RUN_TO_POSITION);
    slide.setMode(RUN_TO_POSITION);
    slide2.setMode(RUN_TO_POSITION);

    // Makes the motor brake when no power is applied by shorting out the motor terminals
    pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Sets the maximum power that the motors are allowed to run at
    pivot.setPower(1);
    slide.setPower(1);
    slide2.setPower(1);


    // Mapping limit switch
    limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "limit switch");
    limitSwitch.setMode(DigitalChannel.Mode.INPUT);


    // Maps local variables
    this.limitSlide = limitSlide;
    this.limitPivot = limitPivot;
    this.countsPerDegree = countsPerDegree;
    this.countsPerInch = countsPerInch;

    // Telemetry
    this.telemetry = opMode.telemetry;

    // Allows calling LinearOpMode functions
    myOp = opMode;
  }


  /**
   * Updates pivot and slide target positions
   */
  public void update() {
    pivot.setTargetPosition(pivotTarget);
    slide.setTargetPosition(slideTarget);
    slide2.setTargetPosition(slideTarget);
  }


  /**
   * Homes the arm using the prox sensor for the pivot
   */
  public void homeArm() {
    // Reset timer
    pivotTimer.reset();

    // Makes sure the motors are not moving when starting the function
    pivot.setPower(0);
    slide.setPower(0);
    slide2.setPower(0);

    // Makes it so that motor power can be set directly
    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_USING_ENCODER);
    slide2.setMode(RUN_USING_ENCODER);

    // Move the pivot while the limit switch is not pressed and a timer is not over its limit
    while (!limitSwitch.getState() && pivotTimer.seconds() < 5 && myOp.opModeIsActive()) {
      telemetry.addData("limit switch: ", limitSwitch.getState());
      pivot.setPower(-0.5);
      slide.setPower(-0.3);
      slide2.setPower(-0.3);

      telemetry.addData("Pivot Pos: ", pivot.getCurrentPosition());
      telemetry.update();
    }

    // Resets encoders
    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);
    slide2.setMode(STOP_AND_RESET_ENCODER);

    // Sets mode back to original mode
    pivot.setMode(RUN_USING_ENCODER);
    slide.setMode(RUN_TO_POSITION);
    slide2.setMode(RUN_TO_POSITION);

    // Resets max power for motor to use
    pivot.setPower(1.0);
    slide.setPower(1.0);
    slide2.setPower(1.0);
  }


  /**
   * Sets target position for pivot motor
   *
   * @param x - Target Angle in degrees
   */
  public void setPivot(double x) {

    double current_Angle = pivot.getCurrentPosition();

    // Limits for
    if (current_Angle > limitPivot) {
      x = limitPivot;
      telemetry.addLine("Pivot pos over limit");
    } else if (current_Angle < 0) {
      x = 0;
      telemetry.addLine("Pivot pos under 0");
    }

    pivotTarget = (int) (x * countsPerDegree);
  }


  /**
   * Sets the target length of the arm
   *
   * @param x - Target length
   */
  public void setSlide(int x) {

    // Calculates the max the arm can go without going over the 40IN limit
    double maxLength = limitSlide * Math.cos(Math.toRadians(pivot.getCurrentPosition() / countsPerDegree));
    if (maxLength < 2500)
      maxLength = 2500;


    if (maxLength > limitSlide) maxLength = limitSlide;

    // Clamps the input to maxLength and 0
    if (x > maxLength)
      x = (int) maxLength;
    if (x < 0)
      x = 0;

    slideTarget = x;
  }
}
