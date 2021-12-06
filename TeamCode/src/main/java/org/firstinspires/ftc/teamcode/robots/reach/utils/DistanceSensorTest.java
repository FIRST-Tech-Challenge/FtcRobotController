package org.firstinspires.ftc.teamcode.robots.reach.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {

    DistanceSensor chassisLengthSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        chassisLengthSensor = hardwareMap.get(DistanceSensor.class, "distLength");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("chassis length (m)", chassisLengthSensor.getDistance(DistanceUnit.METER));
            telemetry.update();
        }
    }
}
