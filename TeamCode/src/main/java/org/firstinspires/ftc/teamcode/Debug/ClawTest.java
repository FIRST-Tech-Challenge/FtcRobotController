// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mekanism.Clip;
import org.firstinspires.ftc.teamcode.Mekanism.Grabber;

@TeleOp
public class ClawTest extends LinearOpMode {


  public void runOpMode() throws InterruptedException {

    Clip clip = new Clip(this);
    Grabber grabber = new Grabber(this);

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
    while (opModeIsActive()) {

      if (gamepad2.a && !is2A) {
        game2A = !game2A;
        is2A = true;
      } else if (!gamepad2.a) is2A = false;
      if (game2A)
        clip.setFunnel(1.0);
      else clip.setFunnel(0);


      if (gamepad2.b && !is2B) {
        game2B = !game2B;
        is2B = true;
      } else if (!gamepad2.b) is2B = false;
      if (game2B)
        clip.clamp();
      else clip.unclamp();


      if (gamepad2.x && !is2X) {
        game2X = !game2X;
        is2X = true;
      } else if (!gamepad2.x) is2X = false;
      if (game2X)
        grabber.wristUp();
      else grabber.wristDown();


      grabber.setPower(-gamepad2.left_stick_y);


      clip.update();
      grabber.update();

    }
  }
}
