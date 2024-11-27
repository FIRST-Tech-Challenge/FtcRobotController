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
  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? because if so we need it to move after the waitForStart()
    // or add the movement to the Autonomous
    var swerve = new Swerve(this);
    var arm = new Mekanism(this);

    boolean bPressed = false;
    waitForStart();
    arm.homeArm();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {
      double currentTime = Utils.getTimeSeconds();
      double dt = currentTime - lastTime;
      swerve.teleopDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, dt);
      swerve.periodic();

      // game pad 2
      // sets arm
      if (!gamepad2.a) {
        arm.setSlide(-gamepad2.left_stick_y);
        arm.setPivot(gamepad2.right_stick_y, gamepad2.left_bumper);

        arm.runIntake(gamepad2.left_trigger > .5, gamepad2.right_trigger > .5);
        if (gamepad2.b && !bPressed) {
          arm.toggleWrist();
        }
        bPressed = gamepad2.b;

        if (gamepad2.x) {
          arm.clamp();
        } else if (gamepad2.y) {
          arm.unclamp();
        }
      } else {
        arm.autoClip();
      }

      telemetry.update();

      lastTime = currentTime;
    }
  }
}