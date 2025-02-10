package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "distanceDetectionTest")
public class distanceDetectionTest extends LinearOpMode {
    private BNO055IMU imu;
    private DistanceSensor horizontalDistanceSensor, verticalDistanceSensor;
    private boolean truth;

    public void displayDistance() {
        double horizontalDistance = horizontalDistanceSensor.getDistance(DistanceUnit.CM);
        double verticalDistance = verticalDistanceSensor.getDistance(DistanceUnit.CM);

        telemetry.addData("horiz:", horizontalDistance);
        telemetry.addData("vert:", verticalDistance);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        horizontalDistanceSensor = hardwareMap.get(DistanceSensor.class, "horizontalDistanceSensor");
        verticalDistanceSensor = hardwareMap.get(DistanceSensor.class, "verticalDistanceSensor");

        truth = true;
        waitForStart();
        while (truth) {
            displayDistance();
        }
    }
}