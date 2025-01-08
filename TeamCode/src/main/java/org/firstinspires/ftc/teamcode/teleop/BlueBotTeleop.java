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

  double slideSpeed = 50;

  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? because if so we need it to move after the waitForStart()
    // or add the movement to the Autonomous
    var swerve = new Swerve(this);
    var mek = new Mekanism(this);

    boolean bPressed = false;
    waitForStart();
    mek.homeArm();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {

      // 1. Calculates deltaTime
      double currentTime = Utils.getTimeSeconds();
      double dt = currentTime - lastTime;

      // 2. Moves the robot based on user input
      swerve.teleopDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, dt);
      swerve.periodic();


      if (gamepad2.a) {
        mek.autoClip();

      } else {

        // 1. Sets the target position of the slide, limits set in Mekansim class
        mek.slideTarget += -gamepad2.left_stick_y * slideSpeed;
        if (mek.slideTarget < 0) mek.slideTarget = 0;
        if (mek.slideTarget > mek.limitSlide) mek.slideTarget = mek.limitSlide;
        telemetry.addData("Slide target position: ", mek.slideTarget);

        if (gamepad2.right_bumper) {
          mek.setSlide(0);
          mek.slideTarget = 0;
          sleep(1000);
        }

        // 2. Set the pivot power
        mek.setPivot(-gamepad2.right_stick_y, gamepad2.right_bumper);

        mek.runIntake(gamepad2.right_trigger > .5, gamepad2.left_trigger > .5);
        if (gamepad2.b && !bPressed) {
          mek.toggleWrist();
        }
        bPressed = gamepad2.b;

        if (gamepad2.x) {
          mek.clamp();
        } else if (gamepad2.y) {
          mek.unclamp();
        }
      }


      mek.setSlide((int) mek.slideTarget);









      telemetry.update();
      lastTime = currentTime;
    }
  }
}
