package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.BuildConfig.BUILD_NUMBER;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "TouchSensor, LED and Build_Number Sample", group = "Autonomous")
public class TouchSensorLEDAndBuildNumberSample extends LinearOpMode {
    private TouchSensor button;
    private LED redLed;
    private LED greenLed;

    @Override
    public void runOpMode() {

        // Initialize hardware variables
        button = hardwareMap.get(TouchSensor.class, "button");
 
        redLed = hardwareMap.get(LED.class, "red");
        greenLed = hardwareMap.get(LED.class, "green");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Check if the touch sensor is pressed
            if (button.isPressed()) {
                // Turn on the red LED and turn off the green LED
                redLed.enable(true);
                greenLed.enable(false);
            } else {
                // Turn off the red LED and turn on the green LED
                redLed.enable(false);
                greenLed.enable(true);
            }

            // Send telemetry data to the driver station
            telemetry.addData("Build Number", BUILD_NUMBER);
            telemetry.addData("Touch Sensor Pressed", button.isPressed());
            telemetry.addData("Red LED State", redLed.isLightOn());
            telemetry.addData("Green LED State", greenLed.isLightOn());
            telemetry.update();
        }
    }
}