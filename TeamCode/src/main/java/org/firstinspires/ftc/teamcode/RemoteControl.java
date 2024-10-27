package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class RemoteControl extends LinearOpMode {
  private final ElapsedTime runtime = new ElapsedTime();
  private final Point origin = new Point(0, 0);

  static class Point {
    public double x, y;

    Point(double x, double y) {
      this.x = x;
      this.y = y;
    }

    public double angle() {
      return Math.atan2(this.x, this.y);
    }

    public Point sub(Point point) {
      return new Point(this.x - point.x, this.y - point.y);
    }

    public Point add(Point point) {
      return new Point(this.x + point.x, this.y + point.y);
    }

    public Point divide(double x) {
      return new Point(this.x / x, this.y / x);
    }
  }

  private Point circleToSquare(Point onSquare) {
    double length = Math.max(distance(origin, onSquare), 1.0);

    double u = onSquare.x / length;
    double v = onSquare.y / length;

    double u2 = u * u;
    double v2 = v * v;
    double twoSqrt2 = 2.0 * Math.sqrt(2.0);
    double subTermX = 2.0 + u2 - v2;
    double subTermY = 2.0 - u2 + v2;
    double termX1 = subTermX + u * twoSqrt2;
    double termX2 = subTermX - u * twoSqrt2;
    double termY1 = subTermY + v * twoSqrt2;
    double termY2 = subTermY - v * twoSqrt2;
    double x = 0.5 * Math.sqrt(termX1) - 0.5 * Math.sqrt(termX2);
    double y = 0.5 * Math.sqrt(termY1) - 0.5 * Math.sqrt(termY2);

    return new Point(x, y);
  }

  private double squared(double x) {
    return x * x;
  }

  private double distance(Point one, Point two) {
    return Math.sqrt(squared(one.x - two.x) + squared(one.y - two.y));
  }

  @Override
  public void runOpMode() {
    NinjaBot robot = new NinjaBot(hardwareMap, this);

    double power = 1.0;
    double servo_pos = 0.0;

    waitForStart();
    runtime.reset();

    while (opModeIsActive()) {

      double x = gamepad1.right_stick_x;
      double y = gamepad1.right_stick_y;

      if (gamepad1.x) {
        servo_pos += 0.05;
      }
      if (gamepad1.a) {
        servo_pos -= 0.05;
      }

      Point circle = new Point(x, y);
      Point uv = circleToSquare(circle);
      Point UV = uv.add(new Point(1, 1)).divide(2.0);

      double left = Math.min(distance(origin, circle) * -(1.0 - UV.x - UV.y) * 2.0, 1.0);
      double right = Math.min(distance(origin, circle) * -(UV.x - UV.y) * 2.0, 1.0);

      telemetry.addLine(String.format("\n==== (left %f)", left));
      telemetry.addLine(String.format("\n==== (right %f)", right));
      telemetry.update();

      // set motor power
      robot.leftDrive.setPower(left * power );
      robot.rightDrive.setPower(right * power);

      robot.ladder.setPower(-0.3 * gamepad1.left_trigger);
      robot.ladder.setPower(0.3 * gamepad1.right_trigger);

      robot.claw.setPosition(servo_pos);

      // optional (waits before continuing loop)
      // sleep(10);
    }
  }
}
