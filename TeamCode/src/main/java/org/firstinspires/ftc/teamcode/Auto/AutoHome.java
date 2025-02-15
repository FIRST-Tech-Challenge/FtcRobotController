// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;

@Autonomous(name = "Home arm", preselectTeleOp = "Blue Bot Teleop")
public class AutoHome extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Mekanism mek = new Mekanism(this);
    Swerve drivebase = new Swerve(this);

    drivebase.initGyro();


    waitForStart();
    mek.arm.homeArm();
  }
}
