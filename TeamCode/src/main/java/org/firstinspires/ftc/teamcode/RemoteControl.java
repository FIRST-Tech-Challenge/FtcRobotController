package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class RemoteControl extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();

  // private class Tuple<X, Y> {
  //   public final X x;
  //   public final Y y;
  //   public Tuple(X x, Y y) {
  //     this.x = x;
  //     this.y = y;
  //   }
  // }

  // private Tuple<double, double> circleToSquare(double x, double y) {
  //   double u2 = x * x;
  //   double v2 = y * y;
  //   double twosqrt2 = 2.0 * sqrt(2.0);
  //   double subtermx = 2.0 + u2 - v2;
  //   double subtermy = 2.0 - u2 + v2;
  //   double termx1 = subtermx + x * twosqrt2;
  //   double termx2 = subtermx - x * twosqrt2;
  //   double termy1 = subtermy + y * twosqrt2;
  //   double termy2 = subtermy - y * twosqrt2;
  //   return Tuple(0.5 * sqrt(termx1) - 0.5 * sqrt(termx2), y = 0.5 * sqrt(termy1) - 0.5 * sqrt(termy2);)
  // }

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

      double power = 0.3;

      // set motor power
      robot.leftDrive.setPower(this.gamepad1.left_stick_y * power);

      robot.rightDrive.setPower(this.gamepad1.right_stick_y * power);


      // optional (waits before continueing loop)
      // sleep(10);
    }
  }
}
