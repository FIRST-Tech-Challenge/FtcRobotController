package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="LimitSwitchforSlide", group="Examples")
public class LimitSwitchforSlide extends LinearOpMode {

    // Declare a touch sensor object
    // WE NAMED THE LIMIT SWITCH, "limitSwitch" FOR SIMPLICITY
    private TouchSensor limitSwitch;
    public DcMotor Slide;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        //public boolean isPressed(){return limitSwitch.isPressed();}


        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Print the current state of the touch sensor to the telemetry
            telemetry.addData("Touch Sensor", limitSwitch.isPressed());
            telemetry.update();

            // If the Magnetic Limit Switch is pressed, stop the motor
            if (limitSwitch.isPressed()) {
                Slide.setPower(0);
            } else { // Otherwise, run the motor
                Slide.setPower(0.3);
            }

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