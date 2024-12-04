// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.MathUtil;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.math.controller.SimpleMotorFeedforward;

@TeleOp
public class ResetServos extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    var servos = new Servo[4];
    var encoders = new AnalogInput[4];

    for (int i = 0; i < 4; i++) {
      String name;
      switch (i) {
        case 0 -> name = "FL";
        case 1 -> name = "FR";
        case 2 -> name = "BL";
        case 3 -> name = "BR";
        default -> throw new IndexOutOfBoundsException();
      }

      servos[i] = hardwareMap.servo.get(name + "Servo");
      encoders[i] = hardwareMap.analogInput.get(name + "Encoder");
    }

    waitForStart();
    while (opModeIsActive()) {
      for (int i = 0; i < 4; i++) {
        var servoPos =
            MathUtil.angleModulus(
                (Math.PI * 2) * (encoders[i].getVoltage() / encoders[i].getMaxVoltage()));

        double maxSpeedSecondsPer60Degrees = .14 * .863;
        double maxSteerSpeedRadPerSec = (2 * Math.PI) / (maxSpeedSecondsPer60Degrees * 6);
        SimpleMotorFeedforward steerFeedforward =
            new SimpleMotorFeedforward(0, 1 / maxSteerSpeedRadPerSec);
        servos[i].setPosition((1 - steerFeedforward.calculate(-servoPos)) / 2);
      }
    }
  }
}
