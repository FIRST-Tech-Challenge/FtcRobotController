package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Sensor", group="Demo")
public class DistanceSensorTest extends LinearOpMode{
    // Definitions
    DistanceSensor s_distance;

    @Override
    public void runOpMode() {
        //Initialization code
        s_distance = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Distance",s_distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
