package org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.utils.autonomous.sensors.distance.wrappers.DistanceSensorWrapper;

@Autonomous(name="Regular Distance Sensor Test", group="linear")
public class RegularDistanceSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DistanceSensorWrapper sensor = new DistanceSensorWrapper(hardwareMap,
                hardwareMap.appContext.getString(R.string.NAVIGATION_NORTH_DISTANCE_SENSOR));

        waitForStart();
        while (opModeIsActive()) {
            double distance = sensor.getData();
            telemetry.addData("Distance:", distance);
        }
    }

}
