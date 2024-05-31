package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.*;


@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class RemoteControl extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();

  @Override
  public void runOpMode() {
    NinjaBot robot = new NinjaBot(hardwareMap, this);

    waitForStart();
    runtime.reset();

    // running loop.
    while (opModeIsActive()) {
      // recieve controller inputs
      // eg: boolean move = gamepad1.a;
      boolean left = gamepad1.x;
      boolean right = gamepad1.b;

      // set motor power
      if (left) {
        robot.leftDrive.setPower(1);
      } else {robot.leftDrive.setPower(0);}
      if (right) {
        robot.rightDrive.setPower(1);
      } else {robot.rightDrive.setPower(0);}

      // optional (waits before continueing loop)
      // sleep(10);
    }
  }
}