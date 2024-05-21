package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.*;


@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class Remote_Control extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
      
  @override
  public void runOpMode() {
    NinjaBot robot = new NinjaBot(hardwareMap, this);

    waitForStart();
    runtime.reset();

    // running loop.
    while (opModeIsActive()) {
      // recieve controller inputs
      // eg: boolean move = gamepad1.a;

      // set motor power
      // eg: robot.leftDrive.setPower(1);

      // optional (waits before continueing loop)
      // sleep(10);
    }
  }
}