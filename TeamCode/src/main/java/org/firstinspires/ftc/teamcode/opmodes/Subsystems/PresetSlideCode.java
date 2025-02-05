package org.firstinspires.ftc.teamcode.opmodes.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class PresetSlideCode {

    // Define constants for positions and safety
    final double SLIDE_TICKS_PER_DEGREE =
            145.1 * 5.2 * 5.0 * (1 / 360.0 * 2); // Encoder ticks per degree calculation

    final double MAX_POSITION = (10 * SLIDE_TICKS_PER_DEGREE); // Max position in encoder ticks
    final double HIGH_RUNG_POSITION = -(52.15 * SLIDE_TICKS_PER_DEGREE); // Mid position in encoder ticks
    final double MIN_POSITION = 0.0; // Min position in encoder ticks
    final double LINEAR_SLIDE_POWER = 0.5; // Power applied to the linear slide motor

    private DcMotor linearSlideMotor; // Declare the linear slide motor

    public PresetSlideCode(DcMotor linearSlideMotor) {
        this.linearSlideMotor = linearSlideMotor;

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Method to control the linear slide based on the gamepad input
    public void controlSlide(Gamepad gamepad) {
        // Handle manual control for the linear slide
        if (gamepad.left_trigger > 0.1) {
            // Manual control to move linear slide down
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Check if we're at the minimum position before moving down
            if (linearSlideMotor.getCurrentPosition() > MIN_POSITION) {
                linearSlideMotor.setPower(-LINEAR_SLIDE_POWER); // Move linear slide down
            } else {
                linearSlideMotor.setPower(0); // Stop motor if at min position
            }
        } else if (gamepad.right_trigger > 0.1) {
            // Manual control to move linear slide up
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Check if we're at the max position before moving up
            if (linearSlideMotor.getCurrentPosition() < MAX_POSITION) {
                linearSlideMotor.setPower(LINEAR_SLIDE_POWER); // Move linear slide up
            } else {
                linearSlideMotor.setPower(0); // Stop motor if at max position
            }
        } else if (gamepad.dpad_right) {
            setCurrentPosition(HIGH_RUNG_POSITION);
        } else {
            // Default behavior: holding the position with a small power if no manual input
            int currentPosition = linearSlideMotor.getCurrentPosition();
            linearSlideMotor.setTargetPosition(currentPosition);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Apply a small holding power
            linearSlideMotor.setPower(0.5);

            // Ensure the motor stops at the correct position within valid range
            if (currentPosition >= MAX_POSITION) {
                linearSlideMotor.setPower(0); // Stop at max position
                setCurrentPosition(MAX_POSITION); // Reset to max position
            } else if (currentPosition <= MIN_POSITION) {
                linearSlideMotor.setPower(0); // Stop at min position
                setCurrentPosition(MIN_POSITION); // Reset to min position
            }
        }

        // Prevent encoder overflow
        if (linearSlideMotor.getCurrentPosition() > MAX_POSITION) {
            setCurrentPosition(MAX_POSITION); // Reset to max position
        } else if (linearSlideMotor.getCurrentPosition() < MIN_POSITION) {
            setCurrentPosition(MIN_POSITION); // Reset to min position
        }

        // Telemetry to track encoder values and preset status
        int currentPosition = linearSlideMotor.getCurrentPosition();
        String presetStatus;

/*
        if (currentPosition >= MAX_POSITION) {
            presetStatus = "Max Position";
        } else if (currentPosition <= MIN_POSITION) {
            presetStatus = "Min Position";
        } else if (currentPosition >= MID_POSITION) {
            presetStatus = "Mid Position";
        } else {
            presetStatus = "Between Min and Mid";
        }

        // Display telemetry for the current position and preset status
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Preset", presetStatus);
        telemetry.update(); // Update telemetry (if any) for debugging purposes
 */
    }

    // Method to set the current position of the linear slide motor
    public void setCurrentPosition(double position) {
        // Ensure the position is within the allowed range
        if (position < MIN_POSITION) {
            position = MIN_POSITION;  // Set to minimum if out of range
        } else if (position > MAX_POSITION) {
            position = MAX_POSITION;  // Set to maximum if out of range
        }

        // Set the motor's current position (directly setting encoder position is not usually possible,
        // but we can reset or adjust it via logic)
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset encoder
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Switch back to running with encoder

        // Set the target position (the motor will move to this target position)
        linearSlideMotor.setTargetPosition((int)position);
        linearSlideMotor.setPower(LINEAR_SLIDE_POWER);  // Apply power to move to the target position
    }
}
