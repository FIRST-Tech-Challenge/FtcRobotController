package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Distance Sensor Test", group = "Tests")
public class Test_DistanceSensor extends LinearOpMode {


    public DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");


        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Distance (cm)",
                    String.format("%.02f", distanceSensor.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}

