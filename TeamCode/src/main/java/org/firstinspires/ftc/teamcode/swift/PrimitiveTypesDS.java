package org.firstinspires.ftc.teamcode.swift;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//imports package (teamcode) to code
 @TeleOp()
 public class PrimitiveTypesDS extends OpMode {
 @Override
 public void init() {
  telemetry.addData("Made By","Dawston");
  //digital signature so I know its mine

 int teamNumber = 17348;
 double motorSpeed = 0.5;
 boolean touchSensorPressed = true;
//this is where the variables get equaled to another value

telemetry.addData("Hello","I'm Dawston");
telemetry.addData("Team Number", teamNumber);
telemetry.addData("Motor Speed", motorSpeed);
telemetry.addData("Touch Sensor", touchSensorPressed);
//These are the different variables being called to display on the driver hub
 }

@Override
 public void loop() {

  }
 }
//ends code (code stops parsing)