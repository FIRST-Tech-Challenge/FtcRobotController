package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Gamepad State")
public class GamepadState extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization logic here
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Access the gamepad instance from the FTC SDK

            // Read the state of each button
            boolean aButton = gamepad1.a;
            boolean bButton = gamepad1.b;
            boolean xButton = gamepad1.x;
            boolean yButton = gamepad1.y;

            boolean circle = gamepad1.circle;

            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;

            // Trigger values (float between 0 and 1)
            float leftTrigger = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            boolean leftStickButton = gamepad1.left_stick_button;
            boolean rightStickButton = gamepad1.right_stick_button;

            // Joystick values (float between -1 and 1)
            float leftStickX = gamepad1.left_stick_x;
            float leftStickY = gamepad1.left_stick_y;
            float rightStickX = gamepad1.right_stick_x;
            float rightStickY = gamepad1.right_stick_y;

            // Send telemetry data to the Driver Hub
            telemetry.addData("circle", circle);
            telemetry.addData("A Button", aButton);
            telemetry.addData("B Button", bButton);
            telemetry.addData("X Button", xButton);
            telemetry.addData("Y Button", yButton);
            telemetry.addData("Left Bumper", leftBumper);
            telemetry.addData("Right Bumper", rightBumper);
            telemetry.addData("Dpad Up", dpadUp);
            telemetry.addData("Dpad Down", dpadDown);
            telemetry.addData("Dpad Left", dpadLeft);
            telemetry.addData("Dpad Right", dpadRight);
            telemetry.addData("Left Stick Button", leftStickButton);
            telemetry.addData("Right Stick Button", rightStickButton);
            telemetry.addData("Left Trigger", leftTrigger);
            telemetry.addData("Right Trigger", rightTrigger);
            telemetry.addData("Left Stick", "X: " + leftStickX + " Y: " + leftStickY);
            telemetry.addData("Right Stick", "X: " + rightStickX + " Y: " + rightStickY);

            telemetry.update();

            // Sleep for a short amount of time to prevent excessive CPU usage
            sleep(20);
        }
    }
}
