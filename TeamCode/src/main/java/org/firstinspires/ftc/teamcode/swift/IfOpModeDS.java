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

 if(gamepad1.left_stick_y < 0){
 telemetry.addData("Left stick", " is negative");
 }
  if(gamepad1.right_stick_y < 0){
   telemetry.addData("Right stick", " is negative");
  }

telemetry.addData("Left stick y", gamepad1.left_stick_y);
//this is whats being called to display on the driver hub. If "if" is true, then it runs whatever is in "if"
 if(gamepad1.a) {
  telemetry.addData("Crazy Mode", "Activated");
  telemetry.addData("Right stick y", gamepad1.left_stick_y);
  telemetry.addData("Left stick y", gamepad1.right_stick_y);
 }
  else{
  telemetry.addData("Crazy Mode", "Deactivated");}
 if(gamepad1.b) {
  telemetry.addData("Turbo Mode", "Activated");

 }
 else {
  telemetry.addData("Turbo Mode", "Deactivated");
 }
//same here
   }
 }
//ends code (code stops parsing)