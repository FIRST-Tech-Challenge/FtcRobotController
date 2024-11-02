// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class ServoTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Servo[] servos = new Servo[4];

    for (int i = 0; i < 4; i++) {
      String pos;
      switch (i) {
        case 0 -> pos = "FL";
        case 1 -> pos = "FR";
        case 2 -> pos = "BL";
        case 3 -> pos = "BR";
        default -> throw new IllegalArgumentException("Module ID is out of range 0-3!");
      }
      servos[i] = hardwareMap.servo.get(pos + "Servo");
    }

    waitForStart();
    while (opModeIsActive()) {
      for (var servo : servos) {
        servo.setPosition((gamepad1.left_stick_y + 1) / 2);
      }
    }
  }
}
