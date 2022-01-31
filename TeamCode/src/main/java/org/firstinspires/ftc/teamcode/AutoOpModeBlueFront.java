// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Auto OpMode BlueFront", group="16481")
public class AutoOpModeBlueFront extends LinearOpMode {

   robot16481 robot = new robot16481();

   @Override
   public void runOpMode() throws InterruptedException {

      robot.telemetry = telemetry;
      robot.hwMap = hardwareMap;
      robot.hardwareSetup();

      robot.initTfod();

      int postion=1;

      while (!(isStarted() || isStopRequested())) {
         idle();
      }

      while (opModeIsActive()) {

         robot.setClawPosition(robot.CLAW_CLOSE);
         robot.tbServo.setPosition(1);

         // 1 sec wait time for object detection is need not sure why. If not it is not working
         Thread.sleep(1000);
         postion  = robot.elementPosition();
         telemetry.addData("# postion", postion);
         telemetry.update();
         robot.setArm(3);
         // Thread.sleep(500);

         // Move away from wall
         robot.sensorDrive(robot.FIRST_SEG);
         //Driver to carousel
         robot.sensorDrive(robot.SECOND_SEG);
         robot.carouselAuto(1);
         //Move back in front of shipping hub
         robot.sensorDrive(robot.THIRD_SEG);
         robot.setArm(postion);
         if(postion==1)
            robot.sensorDrive(robot.FORTH_SEG_1);
         else if (postion==2)
            robot.sensorDrive(robot.FORTH_SEG_2);
         else if (postion==3)
            robot.sensorDrive(robot.FORTH_SEG_3);
         robot.setClawPosition(robot.CLAW_OPEN);

         robot.sensorDrive(robot.FIFTH_SEG);
         robot.sensorDrive(robot.SIXTH_SEG);
         /*
         robot.encoderDrive(20,20,1, robot.STRAFE_RIGHT);
         robot.encoderDrive(41,41,1, robot.FORWARD);
         robot.encoderDrive(20,20,1, robot.STRAFE_LEFT);
         robot.encoderDrive(45,45,1, robot.FORWARD);
         */
         // robot.sensorDrive2();

         /*
         robot.encoderDrive(37,37,1, robot.FORWARD);
         robot.setArm(3);
         robot.setClawPosition(robot.clawOpen);
         robot.encoderDrive(50,50,1, robot.FORWARD);
         */
         break;
      }
   }
}