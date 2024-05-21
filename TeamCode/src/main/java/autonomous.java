package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous
public class Autonomous extends LinearOpMode {
  @Override
  public void runOpMode() {
    NinjaBot robot = new NinjaBot(hardwareMap, this);

    // code to run before the start of the match

    waitForStart();

    // there is not running loop, all code is executed sequentially.
    // same as remote control, but without the loop.
    // eg: robot.leftDrive.setPower(1);
  }
}