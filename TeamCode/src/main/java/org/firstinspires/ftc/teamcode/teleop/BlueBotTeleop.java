// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Auto.AutoSwerve;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  double slideSpeed = 80;
  GoBildaPinpointDriver odometry;

  AutoSwerve driveBase;
  boolean d_pad = false;
  boolean lastPressed = false;

  public double frOffset = -0.125;
  public double brOffset = -0.125;
  public double blOffset = -0.25;
  public double flOffset = -0.25;
  public double frRotationOffset = -0.125;
  public double brRotationOffset = -0.125;
  public double blRotationOffset = -0.25;
  public double flRotationOffset = -0.25;

  public final double change_In_Offset = .025;

  double normalize_angle(double angle) {
      angle += (Math.PI / 2.0);
      
      // If the robot is turning right, the theta angle can become negative.
      // The angle should remain within the range [0,2*pi).
      while (angle < 0.0) {
        angle += 2 * Math.PI;
      }

      return angle;
  }

  enum SteerDirection {
    left,
    right
  };

  private SteerDirection get_steer_direction(double steering_angle) {
    if (steering_angle > 0.5) {
      return SteerDirection.left;
    }
    else {
      return SteerDirection.right;
    }
  }

  /**
   * Returns boolean based on a software implementation of a Schmitt trigger.
   */
  private boolean get_drive_direction(double steering_angle, boolean current_drive_direction, double schmitt_breakpoint_tolerance) {
    boolean updated_drive_direction = current_drive_direction;
    SteerDirection steer_direction = get_steer_direction(steering_angle);

    if (steer_direction == SteerDirection.left) {
      telemetry.addLine("Steer dir: LEFT");
      // Check if steering_angle is above the top threshold to result in a shift to drive forward
      if (steering_angle < (0.75 - schmitt_breakpoint_tolerance)) {
        updated_drive_direction = true;
      }
      // Check if steering_angle is below the bottom threshold to result in a shift to drive backwards
      else if (steering_angle > (0.75 + schmitt_breakpoint_tolerance)) {
        updated_drive_direction = false;
      }
    }
    else {
      telemetry.addLine("Steer dir: RIGHT");
      // Check if steering_angle is above the top threshold to result in a shift to drive forward
      if (steering_angle > (0.25 + schmitt_breakpoint_tolerance)) {
        updated_drive_direction = true;
      }
      // Check if steering_angle is below the bottom threshold to result in a shift to drive backwards
      else if (steering_angle < (0.25 - schmitt_breakpoint_tolerance)) {
        updated_drive_direction = false;
      }
    }

    return updated_drive_direction;
  }

  enum GeneralDirection {
    UP,
    DOWN,
    LEFT,
    RIGHT
  }

  /**
   * Determines the general direction of the robot strafe.
   * General in this case is either up, down, left, or right.
   * This function is used to determine which two wheels should be the drive wheels
   * during a turn.
   */
  private GeneralDirection get_general_direction(double steering_angle) {
    double north = 0.5;
    double east = 0.25;
    double west = 0.75;

    double northEast = (east + north) / 2.0;
    double northWest = (north + west) / 2.0;
    double southWest = (west + 1.0) / 2.0;
    double southEast = (0.0 + east) / 2.0;

    if (steering_angle <= northEast && steering_angle >= southEast) {
      return GeneralDirection.RIGHT;
    }
    else if (steering_angle <= northWest && steering_angle >= northEast) {
      return GeneralDirection.UP;
    }
    else if (steering_angle >= northWest && steering_angle <= southWest) {
      return GeneralDirection.LEFT;
    }
    else {
      return GeneralDirection.DOWN;
    }
  }

  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? not anymore but you need to init the wrist or press b to get it to go to the right position
    Mekanism mek = new Mekanism(this);

    double slideSpeed = 100;
    boolean bPressed = false;

    Init();
    waitForStart();
    mek.arm.homeArm();
    mek.grabber.initWrist();

    double previous_steer_direction = 0.5;
    double previous_driving_speed = 0.0;

    odometry.resetHeading();
    odometry.resetPosAndIMU();
    steer_wheels(previous_steer_direction);

    // Direction of drive, true = forward, false = reverse
    boolean drive_direction = true;

    // The tolerance of the schmitt trigger from y-axis set to zero.
    double schmitt_breakpoint_tolerance = 0.05;

    while (opModeIsActive()) {
      
      // Read raw input from left joystick
      double left_joy_x = gamepad1.left_stick_x;
      double left_joy_y = gamepad1.left_stick_y;

      // Reverse joystick y-axis to ensure 1 is up and -1 is down
      left_joy_y *= -1;

      telemetry.addLine("Left joy x: " + left_joy_x);
      telemetry.addLine("Left joy y: " + left_joy_y);

      // Compute theta of the left joystick vector
      double joy_theta = Math.atan2(left_joy_y, left_joy_x);

      telemetry.addLine("Joy theta: " + joy_theta);

      // Compute vector magnitude, this will be the speed of the robot
      double joy_magnitude = Math.sqrt(Math.pow(left_joy_x, 2.0) + Math.pow(left_joy_y, 2.0));

      telemetry.addLine("Joy mag: " + joy_magnitude);

      // Read raw value from odometer (in radians)
      double robot_theta = odometry.getHeading().getRadians();
      odometry.update();

      // Normalize odometry reading to align with standard polar coordinates
      robot_theta = normalize_angle(robot_theta);
      telemetry.addLine("Robot theta: " + robot_theta);

      // Compute steering angle relative to field-centric movements
      double steering_angle = joy_theta - robot_theta;
      if (steering_angle < -Math.PI) {
        steering_angle += 2*Math.PI;
      }
      steering_angle = steering_angle % (2.0 * Math.PI);

      telemetry.addLine("Chang in steering (from robot heading): " + steering_angle);

      // Normalize steering to 0 to 1 range
      steering_angle /= Math.PI;

      telemetry.addLine("Normalized steering: " + steering_angle);

      //set steering angle to usable servo value
      steering_angle = (steering_angle + .5) / 2 + .25;
      if(joy_magnitude<0.01)
        steering_angle = 0.5;

      telemetry.addLine("Servo steering: " + steering_angle);

      /**
       * At this point, we have vector magnitudes and angles associated with both the 
       * joystick input and the robot's odometry. The robot's odometry operates on the 
       * unit circle, meaning its vector magnitude is irrelevant for determining field-centric 
       * control. This is why simply computing the vector sum is insufficient for accurate 
       * field-centric steering adjustments.
       * 
       * The `steering_angle` variable represents the actual orientation of the swerve 
       * module wheels required for field-centric strafing. However, a key challenge arises 
       * because the swerve drive modules have a limited steering range—typically 270 degrees. 
       * If the wheels attempt to rotate past this limit to align with the intended direction, 
       * they will suddenly rotate the full 270 degrees to reach the target angle from the 
       * opposite side.
       * 
       * One way to address this issue is to constrain steering adjustments to a 180-degree 
       * range. If the computed angle exceeds this threshold, the direction should be inverted, 
       * and the drive motor should reverse. This method works effectively as long as the robot 
       * isn't continuously strafing left and right. Otherwise, a hysteresis effect occurs, 
       * causing oscillations when the wheels attempt to realign near the 180-degree boundary.
       * 
       * To mitigate this, a software-based Schmitt trigger can be implemented. By defining 
       * upper and lower threshold limits, the system can establish a stable directional state, 
       * reducing unwanted oscillations. The `steering_angle` variable should be used to 
       * determine transitions between positive and negative directions to maintain smooth 
       * and predictable steering behavior.
       *
       * 
       * The `steering_angle` variable represents the steering direction relative to the robot's frame.
       * The values are mapped as follows:
       *
       *    Direction   | steering_angle Value
       *  ------------------------------------
       *    Forward     | 0.5
       *    Left        | 0.75
       *    Right       | 0.25
       *    Reverse     | 0 or 1
       */

      boolean previous_drive_direction = drive_direction;
      drive_direction = get_drive_direction(steering_angle, drive_direction, schmitt_breakpoint_tolerance);

      if (drive_direction) {
        telemetry.addLine("POS");
      } else {
        telemetry.addLine("NEG");
      }

      // At this point we're assuming that we're moving forward. See if the schmitt direction
      // needs to move backwards, if so set to reverse. 
      if (drive_direction == false) {
        // If wheels are facing left
        if (steering_angle > 0.5) {
          steering_angle = (steering_angle - 0.75) + 0.25;
        }
        else {
          steering_angle = 0.75 - (0.25 - steering_angle);
        }

        joy_magnitude *= -1.0;
      }
      telemetry.addLine("New theta: " + steering_angle);


      // The computed vector exceeds the allowable drive speed, cap the speed where needed
      double actual_wheel_speed;
      if (joy_magnitude > 1.0) {
        actual_wheel_speed = 1.0;
      }
      else if (joy_magnitude < -1.0) {
        actual_wheel_speed = -1.0;
      }
      else {
        actual_wheel_speed = joy_magnitude;
      }

      /**
       * To this point, the code supports strafe. Steering can be determined by taking into
       * consideration the drive speed and the value of the right joystick x-axis.
       * Multiplying by -1 is because smaller numbers of steering_angle are on the right
       * side whereas -1 is on the left side of the x-axis joystick.
       */
      double right_joy_x = gamepad1.right_stick_x * -1.0;
      double max_turn_radius;
      
      // If the speed decreases enough, allow turns to be sharper. These values are the max
      // the wheels can turn (if the right joystick is held all the way to one side)
      if (Math.abs(actual_wheel_speed) < 0.5) {
        max_turn_radius = 0.15;
      }
      else {
        max_turn_radius = 0.1;
      }

      // Compute the turning radius
      double right_joystick_steering_amt = right_joy_x * max_turn_radius;
      
      telemetry.addLine("Rt joystick raw: " + right_joy_x);
      telemetry.addLine("Rt joystick steer: " + right_joystick_steering_amt);
      telemetry.addLine("Actual speed: " + actual_wheel_speed);

      if ((actual_wheel_speed > 0.001) || (actual_wheel_speed < -0.001)) {
        // Here we're moving the robot in some direction. The heading can be simplified
        // as generally strafing (relative to the front of the robot) up, down, left, or right.
        // That general direction implies that different pairs of wheels are driving the robot.
        // That wheel pair needs to turn into the turn and the other two wheels drive out of
        // the turn.
        // As an example, if the robot is moving forward, the front left and right wheels should
        // steer into the turn and the back left and right wheels should steer awy from the turn.
        // When the robot is moving right, the right two wheels should steer into the turn and 
        // the left two wheels should steer out of the turn.

        // Determine the general direction of the robot
        GeneralDirection general_direction = get_general_direction(steering_angle);

        // Strafing up or down means the front two wheels are the driving wheels.
        // This works in reverse because of the simple direction change.
        if (general_direction == GeneralDirection.UP || general_direction == GeneralDirection.DOWN) {
          steer_wheels(
            steering_angle + right_joystick_steering_amt,
            steering_angle + right_joystick_steering_amt,
            steering_angle - right_joystick_steering_amt,
            steering_angle - right_joystick_steering_amt
          );
        }
        // Strafing left means the left front and back wheels are now the driving wheels
        else if (general_direction == GeneralDirection.LEFT) {
          steer_wheels(
            steering_angle + right_joystick_steering_amt, // Front left
            steering_angle - right_joystick_steering_amt, // Front right
            steering_angle + right_joystick_steering_amt, // Back left
            steering_angle - right_joystick_steering_amt  // Back right
          );
        }
        // Strafing right means the right front and back wheels are now the driving wheels
        else {
          steer_wheels(
            steering_angle - right_joystick_steering_amt, // Front left
            steering_angle + right_joystick_steering_amt, // Front right
            steering_angle - right_joystick_steering_amt, // Back left
            steering_angle + right_joystick_steering_amt  // Back right
          );
        }

        // Reduce drive speed to 75%
        actual_wheel_speed *= 0.75;
        drive_wheels(actual_wheel_speed);
      }
      // Determine if the robot should turn in place
      else {
        // See if both joysticks are at center position, if so steer wheels
        // to default position. The 0.001 here is because doubles in java are dumb.
        // This is a way to check if the value is equal to 0.0 with some rounding
        // issues.
        if (right_joy_x < 0.001 && right_joy_x > -0.001) {
          // keep the steering in place to allow for finer movements up to a target.
          // But stop the wheels.
          drive_wheels(0.0);
        }
        else {
          // Turn wheels to central pivot position. The math here (0.25 + 0.5) is done
          // for readability. 0.25 is right, 0.5 is up, therefore if we add those and
          // divide the result by 2, we get the value for the north-east direction.
          steer_wheels(
            (0.25 + 0.5) / 2.0, // Front left
            (0.5 + 0.75) / 2.0, // Front right
            (0.5 + 0.75) / 2.0, // Back left
            (0.25 + 0.5) / 2.0  // Back right
          );

          // Reduce steering power to 75%
          right_joy_x *= 0.75;

          // While turning, the drive of some wheels needs to be reversed.
          // This is because the wheels take the shortest path to get to their 45-degree
          // position, meaning two of them will be backwards from where they should be.
          drive_wheels(
            -1.0 * right_joy_x, // Front left
            right_joy_x,        // Front right
            -1.0 * right_joy_x, // Back left
            right_joy_x         // Back right
          );
        }
      }

      telemetry.update();

    }
  }

  public void Init() {
    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    odometry.recalibrateIMU();
    odometry.resetPosAndIMU();
    odometry.setOffsets(110, 30);
    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
    odometry.setEncoderDirections(FORWARD, FORWARD);
    odometry.resetHeading(Rotation2d.fromDegrees(120));
    driveBase = new AutoSwerve(this, odometry);
  }

  private void steer_wheels(double x_direction) {
    if (Double.isNaN(x_direction)) {
      x_direction = 0.5;
    }
    driveBase.servoBL.setPosition(x_direction);
    driveBase.servoBR.setPosition(x_direction);
    driveBase.servoFL.setPosition(x_direction);
    driveBase.servoFR.setPosition(x_direction);
  }

  private void steer_wheels(double fl, double fr, double bl, double br) {
    if (Double.isNaN(fl) || Double.isNaN(fr) || Double.isNaN(bl) || Double.isNaN(br)) {
      fl = 0.5;
      fr = 0.5;
      bl = 0.5;
      br = 0.5;
    }
    driveBase.servoBL.setPosition(bl);
    driveBase.servoBR.setPosition(br);
    driveBase.servoFL.setPosition(fl);
    driveBase.servoFR.setPosition(fr);
  }

  private void drive_wheels(double drive_speed) {
    if (Double.isNaN(drive_speed)) {
      drive_speed = 0.0;
    }
    driveBase.motorBL.setPower(drive_speed);
    driveBase.motorBR.setPower(drive_speed);
    driveBase.motorFL.setPower(drive_speed);
    driveBase.motorFR.setPower(drive_speed);
  }

  private void drive_wheels(double fl, double fr, double bl, double br) {
    if (Double.isNaN(fl) || Double.isNaN(fr) || Double.isNaN(bl) || Double.isNaN(br)) {
      fl = 0.5;
      fr = 0.5;
      bl = 0.5;
      br = 0.5;
    }
    driveBase.motorBL.setPower(bl);
    driveBase.motorBR.setPower(br);
    driveBase.motorFL.setPower(fl);
    driveBase.motorFR.setPower(fr);
  }

  public double fieldOrientedHeading() {
    double heading = odometry.getHeading().getDegrees() / 360;
    return heading;
  }

  public static double getAngle(double x, double y) {
    double ret = 90 - Math.toDegrees(Math.atan2(x, y));
    if (ret < 0)
      ret += 360;
    return ret;
  }
}
