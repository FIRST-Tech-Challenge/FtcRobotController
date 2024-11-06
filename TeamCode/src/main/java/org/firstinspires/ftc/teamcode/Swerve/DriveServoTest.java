// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveServoTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Servo[] servos = new Servo[4];
    AnalogInput[] sensors = new AnalogInput[4];

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
      sensors[i] = hardwareMap.analogInput.get(pos + "Encoder");
    }

    waitForStart();
    while (opModeIsActive()) {
      for (int i = 0; i < 4; i++) {
        String pos;
        boolean run;
        switch (i) {
          case 0 -> {
            pos = "FL";
            run = gamepad1.x;
          }
          case 1 -> {
            pos = "FR";
            run = gamepad1.y;
          }
          case 2 -> {
            pos = "BL";
            run = gamepad1.a;
          }
          case 3 -> {
            pos = "BR";
            run = gamepad1.b;
          }
          default -> throw new IllegalArgumentException("Module ID is out of range 0-3!");
        }

        double commanded = run ? (gamepad1.left_stick_y + 1) / 2 : 0.5;
        servos[i].setPosition(commanded);

        telemetry.addData(
            pos + "/position",
            (sensors[i].getVoltage() / sensors[i].getMaxVoltage()) * 360);
        telemetry.addData(pos + "/commanded", commanded);
      }
      telemetry.update();
    }
  }
}
