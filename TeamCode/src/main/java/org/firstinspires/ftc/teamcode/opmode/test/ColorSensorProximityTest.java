package org.firstinspires.ftc.teamcode.opmode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Proximity Test", group = "Test")
public class ColorSensorProximityTest extends LinearOpMode {

    public ColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {

        DistanceSensor distanceSensorLower = hardwareMap.get(DistanceSensor.class, "dropLowerSensor");
        DistanceSensor distanceSensorUpper = hardwareMap.get(DistanceSensor.class, "dropUpperSensor");

        waitForStart();

        while (!isStopRequested()) {

            telemetry.addData("Lower Distance: ", distanceSensorLower.getDistance(DistanceUnit.MM));
            telemetry.addData("Upper Distance: ", distanceSensorUpper.getDistance(DistanceUnit.MM));

            telemetry.update();

        }
    }
}