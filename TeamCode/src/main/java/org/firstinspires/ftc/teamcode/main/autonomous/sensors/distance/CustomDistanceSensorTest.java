package org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.autonomous.sensors.distance.UltrasonicDistanceSensor;

@Autonomous(name="Custom Distance Sensor Test", group="linear")
public class CustomDistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        UltrasonicDistanceSensor sensor = new UltrasonicDistanceSensor(hardwareMap,
                hardwareMap.appContext.getString(R.string.NAVIGATION_NORTH_DISTANCE_SENSOR), this);

        waitForStart();
        while (opModeIsActive()) {
            int distance = sensor.getData();
            telemetry.addData("", distance);
            telemetry.update();
        }
    }

}
