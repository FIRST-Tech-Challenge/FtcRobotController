package org.firstinspires.ftc.teamcode.swift;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//imports package (teamcode) to code
 @TeleOp()
 public class IfOpModeDS extends OpMode {
 @Override
 public void init() {
  telemetry.addData("Made By","Dawston");
  //digital signature so I know its mine
 }

 @Override
 public void loop() {

  double speedForward = -gamepad1.left_stick_y * .5;

  // the individual values is what will pop up here when you move the joysticks

 if(gamepad1.left_stick_y < 0){
 telemetry.addData("Left stick", " is negative");
 }
  if(gamepad1.right_stick_y < 0){
   telemetry.addData("Right stick", " is negative");
  }

  if(gamepad1.b) {
   telemetry.addData("Turbo Mode", "Activated");
   speedForward *= 2;

  }
  else {
   telemetry.addData("Turbo Mode", "Deactivated");
  }
//same here

//this is whats being called to display on the driver hub. If "if" is true, then it runs whatever is in "if"
 if(gamepad1.a) {
  telemetry.addData("Crazy Mode", "Activated");
  telemetry.addData("Left stick x", speedForward);
  telemetry.addData("Left stick y", gamepad1.left_stick_x);
 }
  else{
  telemetry.addData("Crazy Mode", "Deactivated");
  telemetry.addData("Left stick y", speedForward);
  telemetry.addData("Left stick x", gamepad1.left_stick_x);
 }

   }
 }
//ends code (code stops parsing)