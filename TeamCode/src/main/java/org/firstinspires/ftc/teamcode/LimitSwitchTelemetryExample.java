package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="TouchSensorTelemetryExample", group="Examples")
public class LimitSwitchTelemetryExample extends LinearOpMode {

    // Declare a touch sensor object
    private TouchSensor touchSensor;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Print the current state of the touch sensor to the telemetry
            telemetry.addData("Touch Sensor", touchSensor.isPressed());
            telemetry.update();
        }
    }
}




/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="LimitSwitchTelemetryExample", group="Examples")
public class LimitSwitchTelemetryExample extends LinearOpMode {

    // Declare a limit switch object
    private DigitalChannel limitSwitch;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        // Set the limit switch to be an input
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Print the current state of the limit switch to the telemetry
            telemetry.addData("Limit Switch", limitSwitch.getState());
            telemetry.update();
        }
    }
}

*/