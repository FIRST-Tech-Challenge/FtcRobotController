package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers.UltrasonicDistanceSensor;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

@Autonomous(name="3 Distance Sensor Test", group="linear")
public class MultipleDistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        UltrasonicDistanceSensor sensor1 = new UltrasonicDistanceSensor(hardwareMap,
                Resources.Navigation.Sensors.Distance.North, this);
        UltrasonicDistanceSensor sensor2 = new UltrasonicDistanceSensor(hardwareMap,
                Resources.Navigation.Sensors.Distance.East, this);
        UltrasonicDistanceSensor sensor3 = new UltrasonicDistanceSensor(hardwareMap,
                Resources.Navigation.Sensors.Distance.West, this);

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
