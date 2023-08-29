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
telemetry.addData("Left stick y", gamepad1.left_stick_y);
//this is whats being called to display on the driver hub. If "if" is true, then it runs whatever is in "if"
 if(gamepad1.a)
         telemetry.addData("A Button", "pressed");
//same here
   }
 }
//ends code (code stops parsing)