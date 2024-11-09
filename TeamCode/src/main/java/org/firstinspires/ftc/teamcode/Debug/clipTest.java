// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class clipTest extends LinearOpMode {

  Servo ramp1, ramp2;

  @Override
  public void runOpMode() throws InterruptedException {
    initRobot();
    waitForStart();
    while (opModeIsActive()) {
      if (gamepad1.a) {
        ramp1.setPosition(0);
        ramp2.setPosition(.15);
      } else if (gamepad1.b) {
        ramp1.setPosition(.15);
        ramp2.setPosition(0);
      }
    }
  }

  public void initRobot() {
    ramp1 = hardwareMap.get(Servo.class, "ramp 1");
    ramp2 = hardwareMap.get(Servo.class, "ramp 2");

    ramp1.setDirection(Servo.Direction.FORWARD);
    ramp2.setDirection(Servo.Direction.FORWARD);

    ramp1.setPosition(0);
    ramp2.setPosition(0.15);

    ramp1.scaleRange(0, 10);
    ramp2.scaleRange(0, 10);
  }
}
