// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestCarouselSensor", group = "16481")
public class testCarouselSensor extends LinearOpMode {

   DcMotor carouselMotor;
   DistanceSensor sensorRange;
   double distance=0;

   @Override
   public void runOpMode() throws InterruptedException {

      carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
      sensorRange = hardwareMap.get(DistanceSensor.class, "carouselRange");


      waitForStart();

      if (isStopRequested()) return;

      while (opModeIsActive()) {



         distance = sensorRange.getDistance(DistanceUnit.MM);
         if (distance < 60.00)
         {
            carouselMotor.setPower(0.5);
         }
         else
         {
            carouselMotor.setPower(0.0);
         }
         telemetry.addData("# Range Sensor Value:", distance);
         telemetry.update();
      }
   }
}