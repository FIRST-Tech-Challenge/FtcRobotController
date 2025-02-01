// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Utils;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  double slideSpeed = 100;

  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? because if so we need it to move after the waitForStart()
    // or add the movement to the Autonomous
    Swerve swerve = new Swerve(this);
    Mekanism mek = new Mekanism(this);

    boolean bPressed = false;
    waitForStart();
    //mek.homeArm();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {

      double strafe_joystick = gamepad1.left_stick_x;
      double drive_joystick = -1 * gamepad1.left_stick_y;

      double vector_angle = Math.atan2(drive_joystick, strafe_joystick) * (180.0 / Math.PI);
      double vector_length = Math.sqrt(Math.pow(strafe_joystick, 2.0) + Math.pow(drive_joystick, 2.0));

      vector_angle = (vector_angle + 90.0) % 360.0;

      // Normalize so the angle works with AutoSwerve
      vector_angle /= 360.0;

      if (vector_angle < 0) {
        vector_angle = Math.abs(1.0 + vector_angle);
      }
      
      telemetry.addLine("Drive:        " + drive_joystick);
      telemetry.addLine("Strafe:       " + strafe_joystick);
      telemetry.addLine("Vector len:   " + vector_length);
      telemetry.addLine("Vector angle: " + vector_angle);

      telemetry.update();
    }
  }
}
