package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers.UltrasonicDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

@Autonomous(name="Custom Distance Sensor Test", group="linear")
public class CustomDistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        UltrasonicDistanceSensor sensor = new UltrasonicDistanceSensor(hardwareMap,
                Resources.Navigation.Sensors.Distance.North, this);

        waitForStart();
        while (opModeIsActive()) {
            int distance = sensor.getData();
            telemetry.addData("", distance);
            telemetry.update();
        }
    }

}
