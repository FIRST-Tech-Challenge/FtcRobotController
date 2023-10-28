package org.firstinspires.ftc.teamcode.swift;

 import com.qualcomm.robotcore.eventloop.opmode.Disabled;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//imports package (teamcode) to code
 @TeleOp()
 @Disabled
 public class UseStringDS extends OpMode {
 @Override
 //extends the basic OpMode and overrides it from changing Main
 public void init() {
     telemetry.addData("Made By","Dawston");
     //digital signature so I know its mine
     String myName = "Dawston Swift";
     String grade = "10th";
     telemetry.addData("Hello", myName);
     telemetry.addData("Im a ", grade, "grader");
     //this is whats being called to display on the driver hub
 }

 @Override
 public void loop() {

    }
 }
 //ends code (code stops parsing)