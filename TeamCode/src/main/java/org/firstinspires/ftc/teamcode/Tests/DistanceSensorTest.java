package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "Tests")
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensor sensor;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance(mm): ", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
