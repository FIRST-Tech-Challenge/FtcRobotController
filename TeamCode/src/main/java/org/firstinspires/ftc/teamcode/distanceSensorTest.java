package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp (name = "distanceSensorOpMode", group = "sensors9073")

public class distanceSensorTest extends LinearOpMode {
    DistanceSensor distance;
    DcMotor motor;

    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "DistanceTest");
        motor = hardwareMap.get(DcMotor.class, "Motor");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()){
            double distance1 = distance.getDistance(DistanceUnit.CM);
            //Add data and format correctly
            telemetry.addData("status", "running");
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            //Consistently update the data while the Op Mode is running
            telemetry.update();
            }

        }
    }
