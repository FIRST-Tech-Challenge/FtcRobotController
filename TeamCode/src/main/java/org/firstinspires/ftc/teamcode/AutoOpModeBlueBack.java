// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto OpMode Blue Back", group="16481")
public class AutoOpModeBlueBack extends LinearOpMode {

   robot16481 robot = new robot16481();

   @Override
   public void runOpMode() throws InterruptedException {

      robot.telemetry = telemetry;
      robot.hwMap = hardwareMap;
      robot.hardwareSetup();

      robot.initTfod();

      while (!(isStarted() || isStopRequested())) {
         idle();
      }

      while (opModeIsActive()) {

         robot.setClawPosition(robot.CLAW_CLOSE);
         Thread.sleep(1000);
         robot.setArm(robot.elementPosition());

         robot.encoderDrive(19.0, 19.0, 0.5, robot.REVERSE);
         robot.encoderDrive(25, 25, 0.5, robot.STRAFE_RIGHT);
         robot.setClawPosition(robot.CLAW_OPEN);
         robot.encoderDrive(25, 25, 0.5, robot.STRAFE_LEFT);
         robot.encoderDrive(44.0, 44.0, 0.5, robot.FORWARD);

         telemetry.addData("# front left", robot.motorFrontLeft.getPower());
         telemetry.addData("# front right", robot.motorFrontRight.getPower());
         telemetry.addData("# back left", robot.motorBackLeft.getPower());
         telemetry.addData("# back right", robot.motorBackRight.getPower());

         telemetry.addData("pot Value", robot.potMeter.getVoltage());
         telemetry.addData("Servo Value: ", robot.clawServo.getPosition());
         telemetry.update();
         break;
      }
   }
}