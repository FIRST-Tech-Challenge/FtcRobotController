package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "senor_test", group = "")
public class sensor_test extends LinearOpMode
{
    DistanceSensor pole_sensor;
    ColorSensor color_sensor;

    @Override
    public void runOpMode()
    {
        // Get the color sensor from hardwareMap
        pole_sensor  = hardwareMap.get(DistanceSensor.class, "pole_sensor");
        color_sensor = hardwareMap.get(ColorSensor.class,    "pole_sensor");


        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive())
        {
            telemetry.addData("Green", color_sensor.green());
            telemetry.addData("Blue", color_sensor.blue());
            telemetry.addData("Red", color_sensor.red());
            telemetry.addData("Distance", pole_sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}
