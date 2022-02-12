// Created by Vikram Kommera on 10/8/2021
// Mechanum Wheel Control
// Inspired by https://gm0.org/en/latest/docs/software/mecanum-drive.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Test Color Distance Sensor", group = "16481")
public class testColorDistSensor extends LinearOpMode {

   DcMotor carouselMotor;
   RevColorSensorV3 clawObjectSensor;


   double distance=0;

   @Override
   public void runOpMode() throws InterruptedException {

      carouselMotor = hardwareMap.dcMotor.get("carouselMotor");
      clawObjectSensor = hardwareMap.get(RevColorSensorV3.class, "clawObjectSensor");

      waitForStart();

      if (isStopRequested()) return;

      while (opModeIsActive()) {

         telemetry.addData("# Range Sensor Value:", clawObjectSensor.getDistance(DistanceUnit.MM));
         telemetry.update();
      }
   }
}