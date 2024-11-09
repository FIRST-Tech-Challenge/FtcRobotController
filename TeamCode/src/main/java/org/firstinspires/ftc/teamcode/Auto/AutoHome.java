package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;

@Autonomous(name = "Home arm")
public class AutoHome extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    var arm = new Mekanism(this);

    waitForStart();
    arm.homeArm();
  }
}
