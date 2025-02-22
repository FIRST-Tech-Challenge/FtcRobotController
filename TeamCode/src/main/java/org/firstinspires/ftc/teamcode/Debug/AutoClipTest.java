// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;

@TeleOp
public class AutoClipTest extends LinearOpMode {


  public void runOpMode() throws InterruptedException {

    Mekanism mek = new Mekanism(this);

    boolean
        is2A = false,
        is2B = false,
        is2X = false,
        is2Y = false,
        game2A = false,
        game2B = false,
        game2X = false,
        game2Y = false;

    waitForStart();

    mek.arm.homeArm();
    while (opModeIsActive()) {

      if (gamepad2.a && !is2A) {
        game2A = !game2A;
        is2A = true;
      } else if (!gamepad2.a) is2A = false;
      if (game2A)
        mek.clip.setFunnel(1.0);
      else mek.clip.setFunnel(0);


      if (gamepad2.b && !is2B) {
        game2B = !game2B;
        is2B = true;
      } else if (!gamepad2.b) is2B = false;
      if (game2B)
        mek.clip.clamp();
      else mek.clip.unclamp();


      if (gamepad2.x && !is2X) {
        game2X = !game2X;
        is2X = true;
      } else if (!gamepad2.x) is2X = false;
      if (game2X)
        mek.grabber.setWrist(1);
      else mek.grabber.setWrist(0);

      if (gamepad2.dpad_up) {
        mek.clipStage = 1;
      }

      mek.grabber.setGrabber(-gamepad2.left_stick_y, -gamepad2.left_stick_y);

      mek.autoClip();

      telemetry.addLine("Auto Clip Stage: " + mek.clipStage);
      telemetry.addData("pivot current pos", mek.arm.pivot.getCurrentPosition());
      telemetry.addData("slide current pos", mek.arm.slide.getCurrentPosition());

      telemetry.update();
      mek.update();

    }
  }
}
