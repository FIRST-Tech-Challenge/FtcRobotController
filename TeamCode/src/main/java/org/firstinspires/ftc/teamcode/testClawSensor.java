// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestClawSensor", group = "16481")
public class testClawSensor extends LinearOpMode {

   Servo clawServo;
   DistanceSensor sensorRange;
   double distance=0;
   final double clawClose = 0.55;
   final double clawOpen = 0.21;

   @Override
   public void runOpMode() throws InterruptedException {

      clawServo = hardwareMap.servo.get("clawServo");
      sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

      waitForStart();

      if (isStopRequested()) return;

      while (opModeIsActive()) {

         // clawServo.setPosition(position);
         distance = sensorRange.getDistance(DistanceUnit.MM);
         if (distance < 100.00)
         {
            clawServo.setPosition(clawClose);
         }
         else
         {
            clawServo.setPosition(clawOpen);
         }
         telemetry.addData("# Range Sensor Value:", distance);
         telemetry.update();
      }
   }
}