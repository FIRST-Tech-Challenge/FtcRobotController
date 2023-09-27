package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ServoTest extends LinearOpMode {
    // Define the servo object
    private Servo servo;

    // Define a timer to control servo speed
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize the servo object
        servo = hardwareMap.servo.get("servo"); //

        // Set the initial position of the servo
        double currentPosition = 0.0;
        servo.setPosition(currentPosition);

        waitForStart();

        while (opModeIsActive()) {
            // Check if a button on the gamepad is pressed to move the servo
            if (gamepad1.a) {
                // Adjust the servo slowly
                double targetPosition = 1.0;  // Replace with your desired target position (0.0 to 1.0)
                double servoSpeed = 0.02;     // Adjust this value to control the speed of movement

                // Gradually move the servo towards the target position
                if (currentPosition < targetPosition) {
                    currentPosition += servoSpeed;
                } else if (currentPosition > targetPosition) {
                    currentPosition -= servoSpeed;
                }

                // Update the servo position
                servo.setPosition(currentPosition);

                // Reset the timer
                timer.reset();
            }


            // Add other gamepad control logic as needed

            // Sleep briefly to control the loop speed
            sleep(50);
        }
    }
}
