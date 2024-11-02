// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServos extends LinearOpMode {

  Servo FLServo, FRServo, BRServo, BLServo;
  double set;

  @Override
  public void runOpMode() {

    initRobot();
    waitForStart();
    int lastButton = -1;

    while (opModeIsActive()) {

      // inits pow
      double flPow;
      double frPow;
      double blPow;
      double brPow;

      // sets booleans
      if (gamepad1.a) {
        if (lastButton == -1) {
          set += .05;
          lastButton = 1;
        }
      } else if (gamepad1.b) {
        if (lastButton == -1) {
          set -= .05;
          lastButton = 2;
        }
      } else lastButton = -1;

      // if(gamepad1.left_stick_x)

      // sets servos
      FLServo.setPosition(set);
      FRServo.setPosition(set);
      BLServo.setPosition(set);
      BRServo.setPosition(set);

      // sets telemetry
      addTelemetry(
          FLServo.getPosition(),
          FRServo.getPosition(),
          BLServo.getPosition(),
          BRServo.getPosition());
    }
  }

  public void addTelemetry(double flPow, double frPow, double blPow, double brPow) {
    telemetry.addData("FL: ", flPow);
    telemetry.addData("FR: ", frPow);
    telemetry.addData("BL: ", blPow);
    telemetry.addData("BR: ", brPow);
    telemetry.update();
  }

  public void initRobot() {

    FLServo = hardwareMap.get(Servo.class, "FLServo");
    FRServo = hardwareMap.get(Servo.class, "FRServo");
    BLServo = hardwareMap.get(Servo.class, "BLServo");
    BRServo = hardwareMap.get(Servo.class, "BRServo");

    FLServo.setDirection(Servo.Direction.FORWARD);
    FRServo.setDirection(Servo.Direction.FORWARD);
    BLServo.setDirection(Servo.Direction.FORWARD);
    BRServo.setDirection(Servo.Direction.FORWARD);

    FLServo.setPosition(0);
    FRServo.setPosition(0);
    BRServo.setPosition(0);
    BLServo.setPosition(0);

    FLServo.scaleRange(0, 10);
    FRServo.scaleRange(0, 10);
    BLServo.scaleRange(0, 10);
    BRServo.scaleRange(0, 10);

    set = 0;
  }
}
