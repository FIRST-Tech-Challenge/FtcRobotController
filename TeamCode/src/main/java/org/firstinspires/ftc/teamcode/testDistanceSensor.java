// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestRangeSensor", group = "16481")
public class testDistanceSensor extends LinearOpMode {

   DcMotor carouselMotor;
   DistanceSensor leftRange;
   DistanceSensor rightRange;
   DistanceSensor backRange;
   DistanceSensor carouselRange;
   double distance=0;

   @Override
   public void runOpMode() throws InterruptedException {
      carouselRange = hardwareMap.get(DistanceSensor.class, "carouselRange");
      leftRange = hardwareMap.get(DistanceSensor.class, "leftRange");
      rightRange = hardwareMap.get(DistanceSensor.class, "rightRange");
      backRange = hardwareMap.get(DistanceSensor.class, "backRange");

      waitForStart();

      if (isStopRequested()) return;

      while (opModeIsActive()) {
         telemetry.addData("# carousel Range sensor value:", carouselRange.getDistance(DistanceUnit.CM));
         telemetry.addData("# Left Range Sensor Value:", leftRange.getDistance(DistanceUnit.CM));
         telemetry.addData("# Right Range Sensor Value:", rightRange.getDistance(DistanceUnit.CM));
         telemetry.addData("# Back Range Sensor Value:", backRange.getDistance(DistanceUnit.CM));
         telemetry.update();
      }
   }
}