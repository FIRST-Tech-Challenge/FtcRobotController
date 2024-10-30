// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;

@TeleOp
public class ClawTest extends LinearOpMode {

  Mekanism mek = new Mekanism(this);

  public void runOpMode() throws InterruptedException {

    mek.initMekanism();

    waitForStart();
    while (opModeIsActive()) {
      mek.moveClaw(gamepad1.left_trigger);
    }
  }
}
