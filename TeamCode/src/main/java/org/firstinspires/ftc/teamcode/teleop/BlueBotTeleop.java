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

  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? not anymore but you need to init the wrist or press b to get it to go to the right position
    Mekanism mek = new Mekanism(this);

    double slideSpeed = 100;
    boolean bPressed = false;

    Init();
    waitForStart();
    mek.initWrist();
    mek.homeArm();

    double previous_steer_direction = 0.5;
    double previous_driving_speed = 0.0;

    odometry.resetHeading();
    odometry.resetPosAndIMU();
    steer_wheels(previous_steer_direction);

    // Direction of drive, true = forward, false = reverse
    boolean drive_direction = true;

    // The tolerance of the schmitt trigger from y-axis set to zero.
    double schmitt_breakpoint_tolerance = 0.05; // = Math.PI / 8.0;

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

      // joy_theta = normalize_angle(joy_theta, false);

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
        
        double reverse_amt = Math.PI / 8.0;

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

      steer_wheels(steering_angle);

      // Make the wheels move
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

      telemetry.addLine("Actual speed: " + actual_wheel_speed);
      drive_wheels(actual_wheel_speed);

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
    driveBase.servoBL.setPosition(x_direction);
    driveBase.servoBR.setPosition(x_direction);
    driveBase.servoFL.setPosition(x_direction);
    driveBase.servoFR.setPosition(x_direction);
  }

  private void drive_wheels(double drive_speed) {
    driveBase.motorBL.setPower(drive_speed);
    driveBase.motorBR.setPower(drive_speed);
    driveBase.motorFL.setPower(drive_speed);
    driveBase.motorFR.setPower(drive_speed);
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
