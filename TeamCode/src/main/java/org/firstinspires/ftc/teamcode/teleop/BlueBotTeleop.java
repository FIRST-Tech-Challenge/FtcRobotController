//// Copyright (c) 2024-2025 FTC 13532
//// All rights reserved.
//
//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
//import org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver;
//import org.firstinspires.ftc.teamcode.Swerve.Swerve;
//import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
//import org.firstinspires.ftc.teamcode.Utils;
//import org.firstinspires.ftc.teamcode.Auto.AutoSwerve;
//
//import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.EncoderDirection.FORWARD;
//import static org.firstinspires.ftc.teamcode.ODO.GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
//
//@TeleOp(name = "Blue Bot Teleop")
//public class BlueBotTeleop extends LinearOpMode {
//
//  double slideSpeed = 80;
//  GoBildaPinpointDriver odometry;
//
//  AutoSwerve driveBase;
//  boolean d_pad = false;
//  boolean lastPressed = false;
//
//  public double frOffset = -0.125;
//  public double brOffset = -0.125;
//  public double blOffset = 0.5;
//  public double flOffset = 0.25;
//  public double frRotationOffset = -0.3625;
//  public double brRotationOffset = -0.3125;
//  public double blRotationOffset = 0.5;
//  public double flRotationOffset = 0.25;
//
//  @Override
//  public void runOpMode() throws InterruptedException {
//
//    // Does this move the robot? because if so we need it to move after the waitForStart()
//    // or add the movement to the Autonomous
//    Mekanism mek = new Mekanism(this);
//
//    double slideSpeed = 100;
//    boolean bPressed = false;
//    Init();
//    waitForStart();
//    mek.homeArm();
//    double lastTime = Utils.getTimeSeconds();
//    while (opModeIsActive()) {
//
//      // 1. Calculates deltaTime
//      double currentTime = Utils.getTimeSeconds();
//      double dt = currentTime - lastTime;
//
//      // 2.0.1 offsets for swerve (done by holding button and then tapping up or down on the d-pad    TODO: add in adjustable rotation offsets
//      d_pad = gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right;
//      if (gamepad1.a && !lastPressed) {
//        if (d_pad) {
//          if (gamepad1.dpad_up)
//            blOffset += .125;
//          if (gamepad1.dpad_down)
//            blOffset -= .125;
//          if (gamepad1.dpad_left)
//            blRotationOffset -= .0625;
//          if (gamepad1.dpad_right)
//            blRotationOffset += .0625;
//          lastPressed = true;
//        }
//      } else if (gamepad1.b && !lastPressed) {
//        if (d_pad) {
//          if (gamepad1.dpad_up)
//            brOffset += .125;
//          if (gamepad1.dpad_down)
//            brOffset -= .125;
//          if (gamepad1.dpad_left)
//            brRotationOffset -= .0625;
//          if (gamepad1.dpad_right)
//            brRotationOffset += .0625;
//          lastPressed = true;
//        }
//      } else if (gamepad1.x && !lastPressed) {
//        if (d_pad) {
//          if (gamepad1.dpad_up)
//            flOffset += .125;
//          if (gamepad1.dpad_down)
//            flOffset -= .125;
//          if (gamepad1.dpad_left)
//            flRotationOffset -= .0625;
//          if (gamepad1.dpad_right)
//            flRotationOffset += .0625;
//          lastPressed = true;
//        }
//      } else if (gamepad1.y && !lastPressed) {
//        if (d_pad) {
//          if (gamepad1.dpad_up)
//            frOffset += .125;
//          if (gamepad1.dpad_down)
//            frOffset -= .125;
//          if (gamepad1.dpad_left)
//            frRotationOffset -= .0625;
//          if (gamepad1.dpad_right)
//            frRotationOffset += .0625;
//          lastPressed = true;
//        }
//      } else
//        lastPressed = false;
//
//      // 2. takes inputs and makes them work for swerve auto
//      double strafe_joystick = gamepad1.left_stick_x;
//      double drive_joystick = -1 * gamepad1.left_stick_y;
//      double rotate_joystick = gamepad1.right_stick_x;
//      Double vector_angle = Math.atan2(drive_joystick, strafe_joystick) * (180.0 / Math.PI);
//      double vector_length = Math.sqrt(Math.pow(strafe_joystick, 2.0) + Math.pow(drive_joystick, 2.0));
//      vector_angle = (vector_angle + 90.0) % 360.0;
//      // Normalize so the angle works with AutoSwerve
//      vector_angle /= 360.0;
//      if (vector_angle < 0) {
//        vector_angle = Math.abs(1.0 + vector_angle);
//      }
//      vector_angle += 0.125;
////      telemetry.addLine("Drive:        " + drive_joystick);
////      telemetry.addLine("Strafe:       " + strafe_joystick);
////      telemetry.addLine("Vector len:   " + vector_length);
////      telemetry.addLine("Vector angle: " + vector_angle);
////      telemetry.addLine("-------------------------");
//      telemetry.addData("fr offset: ", frOffset);
//      telemetry.addData("fl offset: ", flOffset);
//      telemetry.addData("br offset: ", brOffset);
//      telemetry.addData("bl offset: ", blOffset);
//      telemetry.addData("fr Rotation offset: ", frRotationOffset);
//      telemetry.addData("fl Rotation offset: ", flRotationOffset);
//      telemetry.addData("br Rotation offset: ", brRotationOffset);
//      telemetry.addData("bl Rotation offset: ", blRotationOffset);
//
//      // 2.5 puts values through swerve auto
//      if (vector_angle == null || vector_angle == 0.0)
//        vector_angle = 0.0;
//      if (vector_angle > 1)
//        vector_angle -= 1;
//      if (vector_angle < 0)
//        vector_angle += 1;
//
//      if (vector_angle > .75 && vector_angle < 1.0) {
//        vector_angle -= .5;
//        vector_length *= -1;
//      }
//      if (vector_angle > 0 && vector_angle < 0.25) {
//        vector_angle += .5;
//        vector_length *= -1;
//      }
//
//      if (strafe_joystick != 0. || drive_joystick != 0.) {
//        if (vector_angle > .25 && vector_angle < .75) {
//          driveBase.set_wheels(0, 0, 0, 0, 0);
//        } else {
//          driveBase.set_wheels(
//              vector_angle + frOffset, // Front Right
//              1 - vector_angle - blOffset, // Back Left
//              vector_angle + brOffset, // Back Right
//              -(vector_angle - flOffset), // Front Left
//              fieldOrientedHeading()
//          );
//          driveBase.setMotors(vector_length * 0.5, 0);
//        }
//      } else if (rotate_joystick != 0) {
//        driveBase.steer_wheels_to_central_pivot_position(frRotationOffset, blRotationOffset, brRotationOffset, flRotationOffset);
//        driveBase.setMotors(rotate_joystick * 0.5, 1.0);
//      } else {
//        driveBase.stopServo();
//        driveBase.setMotors(0, 0);
//      }
//
//      if (gamepad2.a) {
//        mek.autoClip();
//        telemetry.addLine("Auto clip");
//      } else {
//
//        // 3. Sets the target position of the slide, limits set in Mekansim class
//        if (-gamepad2.left_stick_y != 0 && gamepad2.left_stick_y!=0.0)
//          mek.slideTarget += -gamepad2.left_stick_y * slideSpeed;
//        if (mek.slideTarget < 0) mek.slideTarget = 0;
//        if (mek.slideTarget > mek.limitSlide) mek.slideTarget = mek.limitSlide;
//        telemetry.addData("Slide target position: ", mek.slideTarget);
//
//        // 3.5 Moves the slide all the way down if right bumper is pressed
//        if (gamepad2.right_bumper) {
//          mek.setSlide(0);
//          mek.slideTarget = 0;
//          sleep(1000);
//        }
//
//        // 4. Set the pivot power
//        mek.setPivot(-gamepad2.right_stick_y, gamepad2.right_bumper);
//
//        // 5. Intake/Outtake control
//        mek.runIntake(gamepad2.left_trigger > .5, gamepad2.right_trigger > .5);
//        if (gamepad2.b && !bPressed) {
//          mek.toggleWrist();
//        }
//        bPressed = gamepad2.b;
//
//        // 6. clamp/unclamp
//        if (gamepad2.x) {
//          mek.clamp();
//        } else if (gamepad2.y) {
//          mek.unclamp();
//        }
//      }
//
//      // 7. Updates the target position of the slide
//      mek.setSlide((int) mek.slideTarget);
//      telemetry.addLine("----------------------------------------");
//      telemetry.addData("X Pos: ", odometry.getPosX());
//      telemetry.addData("Y Pos: ", odometry.getPosY());
//      telemetry.addData("pivot input: ", -gamepad2.right_stick_y);
//      telemetry.addData("pivot pow: ", mek.pivot.getPower());
//      telemetry.addData("field oriented heading: ", fieldOrientedHeading());
//      telemetry.update();
//      lastTime = currentTime;
//      telemetry.update();
//    }
//  }
//
//  public double fieldOrientedHeading() {
//    double heading = odometry.getHeading().getDegrees() / 360;
//    return heading;
//  }
//
//  public void Init() {
//    odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//    odometry.recalibrateIMU();
//    odometry.resetPosAndIMU();
//    odometry.setOffsets(110, 30);
//    odometry.setEncoderResolution(goBILDA_4_BAR_POD);
//    odometry.setEncoderDirections(FORWARD, FORWARD);
//    odometry.resetHeading(Rotation2d.fromDegrees(120));
//    driveBase = new AutoSwerve(this, odometry);
//  }
//}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mekanism.Mekanism;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Utils;

@TeleOp(name = "Blue Bot Teleop")
public class BlueBotTeleop extends LinearOpMode {

  double slideSpeed = 90;
  double pivotSpeed = 100;

  @Override
  public void runOpMode() throws InterruptedException {
    Swerve swerve = new Swerve(this);

    boolean bPressed = false;
//    swerve.initGyro();
    waitForStart();
    Mekanism mek = new Mekanism(this);
    mek.homeArm();
    double lastTime = Utils.getTimeSeconds();
    while (opModeIsActive()) {

      // 1. Calculates deltaTime
      double currentTime = Utils.getTimeSeconds();
      double dt = currentTime - lastTime;

      // 2. Moves the robot based on user input
      swerve.teleopDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, dt);
      swerve.periodic();


      // 1. Sets the target position of the slide, limits set in Mekansim class
      if (-gamepad2.left_stick_y != 0)
        mek.slideTarget += -gamepad2.left_stick_y * slideSpeed;
      if (mek.slideTarget < 0) mek.slideTarget = 0;
      if (mek.slideTarget > mek.limitSlide) mek.slideTarget = mek.limitSlide;
      telemetry.addData("Slide target position: ", mek.slideTarget);

      // 1.5 Moves the slide all the way down if right bumper is pressed
      if (gamepad2.right_bumper) {
        mek.setSlide(0);
        mek.slideTarget = 0;
        sleep(1000);
      }

      // 2. Set the pivot power
      mek.setPivot(-gamepad2.right_stick_y, gamepad2.left_bumper);

      // 3. Intake/Outtake control
      mek.runIntake(gamepad2.left_trigger > .5, gamepad2.right_trigger > .5);
      if (gamepad2.b && !bPressed) {
        mek.toggleWrist();
      }
      bPressed = gamepad2.b;

      // 4. Clamps/unclamps the claw
      if (gamepad2.x) {
        mek.clamp();
      } else if (gamepad2.y) {
        mek.unclamp();
      }

      if(gamepad1.left_bumper) {
        swerve.initGyro();
      }

      // 7. Updates the target position of the slide
      //mek.autoClip();
      mek.setSlide((int) mek.slideTarget);

      telemetry.addData("wrist angle: ",mek.wrist.getPosition());
      telemetry.update();
      lastTime = currentTime;
    }
  }
}
