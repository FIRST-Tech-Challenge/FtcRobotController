package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class test extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
