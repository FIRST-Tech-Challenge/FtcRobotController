package org.firstinspires.ftc.teamcode.Tests;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.TouchSensor;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TouchTest extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor touch;
    DcMotor motor;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        touch = hardwareMap.get(TouchSensor.class, "Touch");

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            // If the touch sensor is pressed, stop the motor
            if (touch.isPressed()) {
                telemetry.addData("pressed?",true);
            } else { // Otherwise, run the motor
                telemetry.addData("pressed?",false);
            }
            telemetry.update();
        }
    }
}
