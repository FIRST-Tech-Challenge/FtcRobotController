package org.firstinspires.ftc.teamcode.swift;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//pulls package from main

 @TeleOp()
 public class GamepadOpMode extends OpMode {
 @Override
 public void init() {
 }
//there is nothing being done in the Init loop

 @Override
 public void loop() {
 telemetry.addData("Left stick x", gamepad1.left_stick_x);
 telemetry.addData("Left stick y", gamepad1.left_stick_y);
 telemetry.addData("A button", gamepad1.a);
 //These three telemetry variables are being sent to display on the drive hub.
  }
}