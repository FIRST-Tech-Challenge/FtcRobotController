// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto OpMode RedFront", group="16481")
public class AutoOpModeRedFront extends LinearOpMode {

   int position=1;
   robot16481 robot = new robot16481();

   @Override
   public void runOpMode() throws InterruptedException {

      robot.telemetry = telemetry;
      robot.hardwareMap = hardwareMap;

      // Very Important: set the side before hardwareSetup
      robot.side = robot.RED;

      robot.hardwareSetup(); // Initialize Hardware
      robot.initTfod(); // Initialize AI objects

      // 1 sec wait time for object detection is need not sure why. If not it is not working
      Thread.sleep(1000);
      position  = robot.elementPosition();
      telemetry.addData("# position", position);
      telemetry.update();

      while (!(isStarted() || isStopRequested())) {  idle();   }

      while (opModeIsActive()) {

         robot.setClawPosition(robot.CLAW_CLOSE);
         Thread.sleep(200);

         // Move arm up
         robot.setArm(2, 0.5);
         Thread.sleep(700);

         // Move arm to front to weight balance
         robot.tbServo.setPosition(0.5);
         Thread.sleep(700);

         // strafe away from wall
         robot.sensorDrive(robot.FIRST_SEG);
         Thread.sleep(500);

         //Drive to carousel
         robot.sensorDrive(robot.SECOND_SEG);
         robot.carouselAuto(1);

         //Move back in front of shipping hub
         robot.sensorDrive(robot.THIRD_SEG);
         Thread.sleep(500);

         // Move toward shipping hub
         robot.sensorDrive(robot.FORTH_SEG);
         Thread.sleep(500);

         // rotate ARM toward shipping hub
         // Blue side tb position is 0
         robot.tbServo.setPosition(0);

         // Move ARM to right height
         robot.setArm(position, 0.5);
         Thread.sleep(500);

         // Final forth segment based on the level to drop
         if      (position==1)  robot.sensorDrive(robot.FORTH_SEG_1);
         else if (position==2)  robot.sensorDrive(robot.FORTH_SEG_2);
         else if (position==3)  robot.sensorDrive(robot.FORTH_SEG_3);

         // Drop the cube into shipping hub
         robot.setClawPosition(robot.CLAW_OPEN);
         Thread.sleep(500);

         robot.sensorDrive(robot.FIFTH_SEG); // strafe back to wall
         robot.sensorDrive(robot.SIXTH_SEG); // Drive to storage
         break;
      }
   }
}