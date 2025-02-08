// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mekanism.Clip;
import org.firstinspires.ftc.teamcode.Mekanism.Grabber;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;

import java.nio.file.attribute.AclEntryFlag;

@TeleOp
public class ClawTest extends LinearOpMode {


  public void runOpMode() throws InterruptedException {

    Clip clip = new Clip(this);
    Grabber grabber = new Grabber(this);

    boolean
        isA = false,
        isB = false,
        game1A = false,
        game1B = false,
        isX = false,
        isY = false,
        game1X = false,
        game1Y = false;

    waitForStart();
    while (opModeIsActive()) {

      if (gamepad1.a && !isA) {
        game1A = !game1A;
        isA = true;
      } else if (!gamepad1.a) {
        isA = false;
      }

      if (game1A)
        clip.setFunnel(1.0);
      else clip.setFunnel(0);


      if (gamepad1.b && !isB) {
        game1B = !game1B;
        isB = true;
      } else if (!gamepad1.b) {
        isB = false;
      }

      if (game1B)
        clip.clamp();
      else clip.unclamp();


      grabber.setPower(-gamepad1.left_stick_y);

      if (gamepad1.x && !isX) {
        game1X = !game1X;
        isX = true;
      } else if (!gamepad1.x) {
        isX = false;
      }

      if (game1X)
        grabber.wristUp();
      else grabber.wristDown();


      clip.update();
      grabber.update();

    }
  }
}
