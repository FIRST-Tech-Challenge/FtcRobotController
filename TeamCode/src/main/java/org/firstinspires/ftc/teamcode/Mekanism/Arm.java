// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Mekanism;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

  LinearOpMode myOp;

  ElapsedTime pivotTimer = new ElapsedTime();

  private final int
      limitSlide = 4200,
      limitPivot = 75;

  private final double
      countsPerDegree = 41.855,
      countsPerInch = 120.88;

  public final DcMotorEx pivot;
  public final DcMotorEx slide, slide2;

  public final DigitalChannel limitSwitch;

  private final Telemetry telemetry;

  private int
      pivotTarget = 0,
      slideTarget = 0;

  private double
      pivotPower = 0,
      slidePower = 0;

  private double maxLength = 0;

  public Arm(LinearOpMode opMode) {

    // Init slaw, claw, and pivot
    pivot = (DcMotorEx) opMode.hardwareMap.dcMotor.get("pivot");
    slide = (DcMotorEx) opMode.hardwareMap.dcMotor.get("slide");
    slide2 = (DcMotorEx) opMode.hardwareMap.get(DcMotor.class, "slide 2");

    // Sets rotation direction for the motors
    pivot.setDirection(FORWARD);
    slide.setDirection(FORWARD);
    slide2.setDirection(FORWARD);

    // Reset all encoders
    pivot.setMode(STOP_AND_RESET_ENCODER);
    slide.setMode(STOP_AND_RESET_ENCODER);
    slide2.setMode(STOP_AND_RESET_ENCODER);

    // Make SURE target position is set before setting the mode to RUN_TO_POSITION
    pivot.setTargetPosition(0);
    slide.setTargetPosition(0);
    slide2.setTargetPosition(0);

    // Set all mek motors RUN_TO_POSITION mode
    pivot.setMode(RUN_TO_POSITION);
    slide.setMode(RUN_TO_POSITION);
    slide2.setMode(RUN_TO_POSITION);

    // Makes the motor brake when no power is applied by shorting out the motor terminals
    pivot.setZeroPowerBehavior(BRAKE);
    slide.setZeroPowerBehavior(BRAKE);
    slide2.setZeroPowerBehavior(BRAKE);

    // Sets the maximum power that the motors are allowed to run at
    pivot.setPower(1);
    slide.setPower(1);
    slide2.setPower(1);


    // Mapping limit switch
    limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "limit switch");
    limitSwitch.setMode(DigitalChannel.Mode.INPUT);

    // Telemetry
    this.telemetry = opMode.telemetry;

    // Allows calling LinearOpMode functions
    myOp = opMode;
  }


  /**
   * Updates pivot and slide target positions
   */
  public void update() {

    if (pivot.getMode() == RUN_TO_POSITION) {
      pivot.setTargetPosition(pivotTarget);
    } else {
      pivot.setPower(pivotPower);
    }

    if (slide.getMode() == RUN_TO_POSITION) {
      slide.setTargetPosition(slideTarget);
      slide2.setTargetPosition(slideTarget);
    } else {
      slide.setPower(slidePower);
      slide2.setPower(slidePower);
    }
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
    // Note: the limit switch is TRUE when it is not activated
    while (limitSwitch.getState() && pivotTimer.seconds() < 5 && myOp.opModeIsActive()) {
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
    pivot.setMode(RUN_TO_POSITION);
    slide.setMode(RUN_TO_POSITION);
    slide2.setMode(RUN_TO_POSITION);

    // Resets max power for motor to use
    pivot.setPower(1.0);
    slide.setPower(1.0);
    slide2.setPower(1.0);
  }


  /**
   * Used to override the default RUN_TO_POSITION mode of the slide and pivot motors
   *
   * @param slideMode Mode of the linear slide motors
   * @param pivotMode Mode of the pivot motor
   */
  public void setRunMode(DcMotor.RunMode slideMode, DcMotor.RunMode pivotMode) {

    if (slideMode == RUN_TO_POSITION) {
      slide.setPower(1);
      slide2.setPower(1);
    } else {
      slide.setPower(0);
      slide2.setPower(0);
    }

    if (pivotMode == RUN_TO_POSITION) {
      pivot.setPower(1);
    } else {
      pivot.setPower(0);
    }

    pivot.setMode(pivotMode);
    slide.setMode(slideMode);
    slide2.setMode(slideMode);
  }


  /**
   * Sets target position for pivot motor if it is in RUN_TO_POSITION mode
   *
   * @param x Target Angle in degrees
   */
  public void setPivot(int x) {

    double current_Angle = pivot.getCurrentPosition() / countsPerDegree;

    // Limits for the motor
    if (current_Angle > limitPivot) {
      x = limitPivot;
      telemetry.addLine("Pivot pos over limit");
    } else if (current_Angle < 0) {
      x = 0;
      telemetry.addLine("Pivot pos under 0");
    }

    pivotTarget = (int) (x * countsPerDegree);

    maxLength = limitSlide * Math.cos(Math.toRadians(pivot.getCurrentPosition() / countsPerDegree)) * 1.4;
    if(slide.getCurrentPosition()>maxLength)
      setSlide(maxLength);
  }


  /**
   * Sets the power of the pivot if it is in RUN_USING_ENCODER mode
   *
   * @param power (-1) to 1
   */
  public void setPivot(double power) {

    double current_Angle = pivot.getCurrentPosition() / countsPerDegree;

    if (current_Angle > limitPivot && power > 0) {
      power = 0;
      telemetry.addLine("Pivot pos over limit");
    } else if (current_Angle < 0 && power < 0) {
      power = 0;
      telemetry.addLine("Pivot pos under 0");
    }

    pivotPower = power / 2;

    maxLength = limitSlide * Math.cos(Math.toRadians(pivot.getCurrentPosition() / countsPerDegree)) * 1;
    if(maxLength < 2500)
      maxLength = 2500;
    if(slide.getCurrentPosition()>maxLength && power > 0) {
      setSlide(-power * 3);
      telemetry.addLine("auto in slide");
    }
  }


  /**
   * Sets the target length of the arm if in RUN_TO_POSITION mode
   *
   * @param x - Target length in encoder counts
   */
  public void setSlide(int x) {

    // Calculates the max the arm can go without going over the 40IN limit
    maxLength = limitSlide * Math.cos(Math.toRadians(pivot.getCurrentPosition() / countsPerDegree)) * 1.4;
    if (maxLength < 2500) maxLength = 2500;

    if (maxLength > limitSlide) maxLength = limitSlide;

    // Clamps the input to maxLength and 0
    if (x > maxLength)
      x = (int) maxLength;
    if (x < 0)
      x = 0;

    slideTarget = x;
  }


  /**
   * Sets the power of the arm if it is in RUN_USING_ENCODER mode
   *
   * @param power (-1) to 1. 0 means the slide will stop, -1 pull the slide in and 1 extends it out.
   */
  public void setSlide(double power) {

    // Calculates the max the arm can go without going over the 40IN limit
    double maxLength = limitSlide * Math.cos(Math.toRadians(pivot.getCurrentPosition() / countsPerDegree)) * 1.2;
    if (maxLength < 2500)
      maxLength = 2500;

    // If the max length calculated is longer than the physical limit of the slide, set it to that
    if (maxLength > limitSlide) maxLength = limitSlide;

    if(slide.getCurrentPosition()>maxLength && power < 0)
      telemetry.addLine("Slide over want to go less: " + power);

    // If the slide goes over the limit, stop the movement
    if (slide.getCurrentPosition() > maxLength && power > 0) {
      power = 0.0;
      telemetry.addLine("Slide over robot length limit");
    } else if (slide.getCurrentPosition() < 0 && power < 0) {
      telemetry.addLine("Slide under 0");
      power = 0;
    }
    telemetry.addLine("Power after checks: " + power);
    slidePower = power;
  }

  public void hang(){
    setSlide(-1);
    setSlide(-10);
  }
}
