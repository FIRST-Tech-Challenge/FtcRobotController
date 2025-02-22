// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

import androidx.lifecycle.GenericLifecycleObserver;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.AutoSwerve;
import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerve.TheBestSwerve;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  double slideSpeed = 80;
  GoBildaPinpointDriver odometry;

  AutoSwerve driveBase;

  boolean
      is2A = false,
      is2B = false,
      is2X = false,
      is2Y = false,
      game2A = false,
      game2B = false,
      game2X = false,
      game2Y = false;

  // TODO: Does this need to be here
  public double frOffset = -0.125;
  public double brOffset = -0.125;
  public double blOffset = -0.25;
  public double flOffset = -0.25;
  public double frRotationOffset = -0.125;
  public double brRotationOffset = -0.125;
  public double blRotationOffset = -0.25;
  public double flRotationOffset = -0.25;

  public final double change_In_Offset = .025;

  @Override
  public void runOpMode() throws InterruptedException {

    // Does this move the robot? not anymore but you need to init the wrist or press b to get it to go to the right position
    Mekanism mek = new Mekanism(this);

    Init();
    waitForStart();
    mek.arm.homeArm();
    mek.grabber.initWrist();
    odometry.resetHeading();
    odometry.resetPosAndIMU();

    TheBestSwerve amazingSwerve = new TheBestSwerve(this,odometry,driveBase);

    mek.arm.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER, DcMotor.RunMode.RUN_USING_ENCODER);

    while (opModeIsActive()) {

      double left_joy_x = gamepad1.left_stick_x;
      double left_joy_y = gamepad1.left_stick_y;

      /**
       * To this point, the code supports strafe. Steering can be determined by taking into
       * consideration the drive speed and the value of the right joystick x-axis.
       * Multiplying by -1 is because smaller numbers of steering_angle are on the right
       * side whereas -1 is on the left side of the x-axis joystick.
       */
      double right_joy_x = gamepad1.right_stick_x * -1.0;

      amazingSwerve.swerveTheThing(left_joy_x, left_joy_y, right_joy_x);

      /*
        Everything before this is for Driving.
        Everything below is for Mekanism
       */

      double
          g2_lx = gamepad2.left_stick_x,
          g2_ly = -gamepad2.left_stick_y,
          g2_rx = gamepad2.right_stick_x,
          g2_ry = -gamepad2.right_stick_y,
          g2_lt = gamepad2.left_trigger,
          g2_rt = gamepad2.right_trigger;

      telemetry.addLine("G2 LY: "+g2_ly);
      telemetry.addLine("G2 RY: "+g2_ry);
      mek.arm.setSlide(g2_ly);
      mek.arm.setPivot(g2_ry);

      // This block handles making the gamepad.b toggle the wrist position
      if (gamepad2.b && !is2B) {
        game2B = !game2B;
        is2B = true;
      } else if (!gamepad2.b) is2B = false;
      if (game2B)
        mek.grabber.setWrist(1.0);
      else mek.grabber.setWrist(-1.0);

      // Grabber power
      double grabberSpeed = g2_lt - g2_rt;
      mek.grabber.setGrabber(grabberSpeed, grabberSpeed);

      mek.update();
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
}
