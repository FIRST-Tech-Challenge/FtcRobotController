package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

@Disabled
@TeleOp(name = "floorDsTest")
public class floorDistanceSensorTest extends GenericOpModeTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "front_distance_sensor");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Distance", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }


    }
}
