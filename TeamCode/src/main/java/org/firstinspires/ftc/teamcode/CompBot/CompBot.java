// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.CompBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "CompBot Swerve", group = "CompBot")
public class CompBot extends LinearOpMode {

  SwerveConfig swerve = new SwerveConfig(this);
  MekanismConfig mek = new MekanismConfig(this);
  Utils utils = new Utils(this);
  GraphicTelemetry graph = new GraphicTelemetry(this);

  Thread mekThread =
      new Thread() {
        public void run() {

          waitForStart();
          while (opModeIsActive()) {

            mek.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mek.slide.setPower(-gamepad2.left_stick_y);

            /*
            boolean clipCanceled = false;

            if (gamepad2.x) {
                mek.armLength(0);
                while (mek.slide.isBusy() && gamepad2.y) idle();

                mek.armAngle(20);
                while (mek.pivot.isBusy() && gamepad2.y) idle();
            }
            mek.pauseSlide();
            mek.pausePivot();
            if (gamepad2.y) clipCanceled = true;

            if (!clipCanceled) {
                mek.armLength(10);
            }
            */
          }
        }
      };

  /**
   * Controls for Gamepad 1: Right trigger: Forwards Left trigger: Reverse Right stick X: Rotate
   * Left stick X Strafe
   *
   * <p>Controls for Gamepad 2: Left stick y: In and out of arm Right stick y: Up and down of arm
   * Left trigger: Claw intake Right trigger: Claw out Presets for: Attaching clip to sample
   * Attaching specimen(clip + sample) to top rung Presets for bucket 1 and 2
   */
  public void runOpMode() throws InterruptedException {

    swerve.initSwerve(); // Inits all the stuff related to swerve drive
    mek.initMekanism(); // Inits the mechanism stuff

    mekThread.start();

    waitForStart();
    while (opModeIsActive()) {
      idle();
    }
  }
}
