package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "TouchSensorTest", group = "")
public class TouchSensorTest extends LinearOpMode
{

    TouchSensor touch;
    @Override
    public void runOpMode ()
    {
        // Get the color sensor from hardwareMap
        touch = hardwareMap.get(TouchSensor.class, "Touch");


        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            if (touch.isPressed())
            {
                telemetry.addData("Touch Sensor Pressed", "");
            }
            else
            {
                telemetry.addData("Touch Sensor Not Pressed", "");
            }
             telemetry.update();
        }

    }
}