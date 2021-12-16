package org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;

@Autonomous(name="3 Distance Sensor Test", group="linear")
public class MultipleDistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        UltrasonicDistanceSensor sensor1 = new UltrasonicDistanceSensor(hardwareMap,
                hardwareMap.appContext.getString(R.string.NAVIGATION_NORTH_DISTANCE_SENSOR), this);
        UltrasonicDistanceSensor sensor2 = new UltrasonicDistanceSensor(hardwareMap,
                hardwareMap.appContext.getString(R.string.NAVIGATION_WEST_DISTANCE_SENSOR), this);
        UltrasonicDistanceSensor sensor3 = new UltrasonicDistanceSensor(hardwareMap,
                hardwareMap.appContext.getString(R.string.NAVIGATION_EAST_DISTANCE_SENSOR), this);

        waitForStart();
        while (opModeIsActive()) {
            int distance = sensor1.getData();
            telemetry.addData("1", distance);

            distance = sensor2.getData();
            telemetry.addData("2", distance);

            distance = sensor3.getData();
            telemetry.addData("3", distance);
            telemetry.update();
        }
    }

}
