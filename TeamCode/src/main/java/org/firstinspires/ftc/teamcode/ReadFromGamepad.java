package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ReadFromGamepad extends LinearOpMode{

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized"); // Telemetry Initialization
        telemetry.update(); // This is how we actually send the telemetry to the phone
        waitForStart(); // The code after this will not run until the start button has been pressed

        // Initialize the values for holding the left and right joystick positions
        // Each joystick is represented with two values: forward/backward position, and left/right position (y and x axis respectively)
        double leftStickForwardBackwards = 0.0;
        double leftStickLeftRight = 0.0;
        double rightStickForwardBackwards = 0.0;
        double rightStickLeftRight = 0.0;

        // runs until the end of the match (until the driver presses STOP)
        while(opModeIsActive()){
            leftStickForwardBackwards = gamepad1.left_stick_y; //These variables come from the class defintion
            leftStickLeftRight = gamepad1.left_stick_x;
            rightStickForwardBackwards = gamepad1.right_stick_y;
            rightStickLeftRight = gamepad1.right_stick_x;

            // Add the values to the telemetry so we can view them on the phone. (%.2f) is for formatting decimal numbers
            telemetry.addData("Left Stick", "Forward/Backwards (%.2f), Right/Left (%.2f)", leftStickForwardBackwards, leftStickLeftRight);
            telemetry.addData("Right Stick", "Forward/Backwards (%.2f), Right/Left (%.2f)", rightStickForwardBackwards, rightStickLeftRight);

            // The buttons can also be found in the class definition
            telemetry.addData("Button A", gamepad1.a);
            telemetry.addData("D-Pad Down", gamepad1.dpad_down);
            telemetry.update(); // Update the telemetry to send the values to the phone
        }
    }
}