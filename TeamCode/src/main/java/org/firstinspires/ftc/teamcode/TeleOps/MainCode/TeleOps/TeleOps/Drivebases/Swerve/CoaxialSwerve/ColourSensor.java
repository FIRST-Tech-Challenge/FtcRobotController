package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Swerve.CoaxialSwerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColourSensor extends LinearOpMode {
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        colorSensor = hardwareMap.get(ColorSensor.class, "Color Sensor");

        // Wait for the start button to be pressed
        waitForStart();

        // Run until the stop button is pressed
        while (opModeIsActive()) {
            // Read the color sensor data
            int redValue = colorSensor.red();
            int greenValue = colorSensor.green();
            int blueValue = colorSensor.blue();

            // Display the detected color on the driver station
            telemetry.addData("Red", redValue);
            telemetry.addData("Green", greenValue);
            telemetry.addData("Blue", blueValue);
            telemetry.update();
        }
    }
}
