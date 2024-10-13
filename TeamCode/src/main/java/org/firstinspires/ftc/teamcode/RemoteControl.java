package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.*;

@TeleOp(name = "Basic: Remote Control", group = "Linear Opmode")
public class RemoteControl extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private Point origin = new Point(0, 0);

  // Transformation
  class Trans {
    public Point i, j;

    Trans(Point i, Point j) {
      this.i = i;
      this.j = j;
    }

    public Point apply(Point point) {
      return new Point(this.i.x * point.x + this.j.x * point.y,
              this.i.y * point.x + this.j.y * point.y);
    }

    public Point inv(Point point) {
      double invDet = (1/(this.i.x * this.j.y - this.j.x * this.i.y));
      // inverse matrix
      Trans inv = new Trans(new Point(invDet * this.j.y, - invDet * this.i.y),
              new Point(- invDet * this.j.x, invDet * this.i.x));
      return inv.apply(point);
    }
  }

  class Point {
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
  }

  private Point circleToSquare(Point onSquare) {
    double u = onSquare.x;
    double v = onSquare.y;

    double u2 = u * u;
    double v2 = v * v;
    double twosqrt2 = 2.0 * Math.sqrt(2.0);
    double subtermx = 2.0 + u2 - v2;
    double subtermy = 2.0 - u2 + v2;
    double termx1 = subtermx + u * twosqrt2;
    double termx2 = subtermx - u * twosqrt2;
    double termy1 = subtermy + v * twosqrt2;
    double termy2 = subtermy - v * twosqrt2;
    double x = 0.5 * Math.sqrt(termx1) - 0.5 * Math.sqrt(termx2);
    double y = 0.5 * Math.sqrt(termy1) - 0.5 * Math.sqrt(termy2);

    return new Point(x, y);
  }

  private double squared(double x) {
    return x * x;
  }

  private double distance(Point one, Point two) {
    return Math.sqrt(squared(one.x - two.x) + squared(one.y - two.y));
  }

  // Ray starts at origin
  private Point projectThrough(Point start, Point through, Point ray, Trans trans) {
    start = trans.inv(start);
    through = trans.inv(through);
    ray = trans.inv(ray);

    // Intercepting ray - ray that intercepts - shooting ray
    double iRayAngle = through.sub(start).angle();
    double c = distance(this.origin, start);

    double C = 180 - (180 - iRayAngle + ray.angle());
    double A = start.angle() - ray.angle();
    double B = 180 - A - C;
    double b = Math.asin(B) * (c / Math.asin(C));

    return trans.apply(new Point(b * Math.cos(ray.angle()), b * Math.sin(ray.angle())));
  }

  private double rightSpeed(Point square, Trans trans) {
    double right = 0;


    return right;
  }

  @Override
  public void runOpMode() {
    NinjaBot robot = new NinjaBot(hardwareMap, this);

    waitForStart();
    runtime.reset();

    while (opModeIsActive()) {
      double power = 0.3;

      double x = this.gamepad1.right_stick_x;
      double y = this.gamepad1.right_stick_y;

      Point circle = new Point(x, y);
      Point square = circleToSquare(circle);
      double sx = square.x;
      double sy = square.y;

      double left = 0;
      double right = 0;

      // Quadrant 2 and 4 get the edges
      if (x > 0 && y > 0) {
        left = distance(this.origin, circle);
        Trans trans = new Trans(new Point(1, 0), new Point(0, 1));
        square = trans.inv(square);

        if (square.y > square.x) {
          Trans subtrans = new Trans(new Point(1, 0), new Point(0, 1));

          Point start = new Point(1, 0);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          right = distance(start, projection) / distance(square, projection);
        } if (square.x > square.y) {
          Trans subtrans = new Trans(new Point(0, 1), new Point(1, 0));

          Point start = new Point(0, 1);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          right = -(distance(start, projection) / distance(square, projection));
        }
      } else if (x <= 0 && y >= 0) {
        right = distance(this.origin, circle);
        Trans trans = new Trans(new Point(-1, 0), new Point(0, 1));
        square = trans.inv(square);

        if (square.y > square.x) {
          Trans subtrans = new Trans(new Point(1, 0), new Point(0, 1));

          Point start = new Point(1, 0);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          left = distance(start, projection) / distance(square, projection);
        } if (square.x > square.y) {
          Trans subtrans = new Trans(new Point(0, 1), new Point(1, 0));

          Point start = new Point(0, 1);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          left = -(distance(start, projection) / distance(square, projection));
        }
      } else if (x < 0 && y > 0) {
        left = -distance(this.origin, circle);
        Trans trans = new Trans(new Point(-1, 0), new Point(0, -1));
        square = trans.inv(square);

        if (square.y > square.x) {
          Trans subtrans = new Trans(new Point(1, 0), new Point(0, 1));

          Point start = new Point(1, 0);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          right = distance(start, projection) / distance(square, projection);
        } if (square.x > square.y) {
          Trans subtrans = new Trans(new Point(0, 1), new Point(1, 0));

          Point start = new Point(0, 1);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          right = -(distance(start, projection) / distance(square, projection));
        }
      } else if (x < 0 && y > 0) {
        right = -distance(this.origin, circle);
        Trans trans = new Trans(new Point(1, 0), new Point(0, -1));
        square = trans.inv(square);

        if (square.y > square.x) {
          Trans subtrans = new Trans(new Point(1, 0), new Point(0, 1));

          Point start = new Point(1, 0);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          left = distance(start, projection) / distance(square, projection);
        } if (square.x > square.y) {
          Trans subtrans = new Trans(new Point(0, 1), new Point(1, 0));

          Point start = new Point(0, 1);
          Point projection = projectThrough(start, square, new Point(1, 1), subtrans);

          left = -(distance(start, projection) / distance(square, projection));
        }
      }

      telemetry.addLine(String.format("\n==== (left %d)", left));
      telemetry.addLine(String.format("\n==== (right %d)", right));

      telemetry.update();
      // set motor power
      robot.leftDrive.setPower(left * power);
      robot.rightDrive.setPower(right * power);

      // optional (waits before continuing loop)
      // sleep(10);
    }
  }
}
