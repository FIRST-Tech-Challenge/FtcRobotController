// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;

@Disabled
@TeleOp
public class ClawTest extends LinearOpMode {
  public void runOpMode() throws InterruptedException {
    Mekanism mek = new Mekanism(this);
    waitForStart();
    while (opModeIsActive()) {
      mek.runIntake(gamepad1.left_trigger > .5, gamepad1.right_trigger > .5);
      if (gamepad1.b) {
        mek.toggleWrist();
      }
    }
  }
}
