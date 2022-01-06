package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers.DistanceSensorWrapper;
import org.firstinspires.ftc.teamcode.main.utils.resources.Resources;

@Autonomous(name="Regular Distance Sensor Test", group="linear")
public class RegularDistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DistanceSensorWrapper sensor = new DistanceSensorWrapper(hardwareMap,
                Resources.Navigation.Sensors.Distance.North);

        waitForStart();
        while (opModeIsActive()) {
            double distance = sensor.getData();
            telemetry.addData("Distance:", distance);
        }
    }

}
