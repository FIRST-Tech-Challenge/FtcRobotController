// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testMotor extends LinearOpMode {

  DcMotor FLMotor, BLMotor, BRMotor, FRMotor, pivot, slide;

  public void runOpMode() {
    initR();

    waitForStart();

    while (opModeIsActive()) {
      BLMotor.setPower(-gamepad1.left_stick_y);

      BRMotor.setPower(-gamepad1.right_stick_y);

      int x = 0;
      if (gamepad1.dpad_up) x = 1;
      else if (gamepad1.dpad_down) x = -1;
      FLMotor.setPower(x);

      x = 0;
      if (gamepad1.y) x = 1;
      else if (gamepad1.a) x = -1;
      FRMotor.setPower(x);

      telemetry.addData("BL motor power: ", BLMotor.getPower());
      telemetry.addData("BR motor power: ", BRMotor.getPower());
      telemetry.addData("FL motor power: ", FLMotor.getPower());
      telemetry.addData("FR motor power: ", FRMotor.getPower());
      telemetry.update();
    }
  }

  public void initR() {
    // Maps the motor objects to the physical ports
    FLMotor = hardwareMap.get(DcMotor.class, "FLMotor");
    BLMotor = hardwareMap.get(DcMotor.class, "BLMotor");
    FRMotor = hardwareMap.get(DcMotor.class, "FRMotor");
    BRMotor = hardwareMap.get(DcMotor.class, "BRMotor");

    // Sets the encoder mode
    FLMotor.setMode(RUN_USING_ENCODER);
    BLMotor.setMode(RUN_USING_ENCODER);
    FRMotor.setMode(RUN_USING_ENCODER);
    BRMotor.setMode(RUN_USING_ENCODER);

    // Sets what happens when no power is applied to the motors.
    // In this mode, the computer will short the 2 leads of the motor, and because of math, the
    // motor will be a lot harder to turn
    FLMotor.setZeroPowerBehavior(BRAKE);
    BLMotor.setZeroPowerBehavior(BRAKE);
    FRMotor.setZeroPowerBehavior(BRAKE);
    BRMotor.setZeroPowerBehavior(BRAKE);

    FLMotor.setDirection(FORWARD);
    BLMotor.setDirection(REVERSE);
    FRMotor.setDirection(REVERSE);
    BRMotor.setDirection(FORWARD);

    FLMotor.setPower(0);
    BLMotor.setPower(0);
    FRMotor.setPower(0);
    BRMotor.setPower(0);
  }
}
