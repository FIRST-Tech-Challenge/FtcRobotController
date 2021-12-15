package org.firstinspires.ftc.teamcode.main.autonomous.location;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.UltrasonicDistanceSensor;

@Autonomous(name="Distance Sensor Test", group="linear")
public class DistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        UltrasonicDistanceSensor sensor = new UltrasonicDistanceSensor(hardwareMap,
                hardwareMap.appContext.getString(R.string.NAVIGATION_FRONT_DISTANCE_SENSOR));

        waitForStart();
        while (opModeIsActive()) {
            double distance = sensor.getData();
            telemetry.addData("Distance:", distance);
        }
    }

}
