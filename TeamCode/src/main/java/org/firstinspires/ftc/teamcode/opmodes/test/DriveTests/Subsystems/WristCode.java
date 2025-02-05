package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class WristCode {

    // Define the Servo for the wrist system
    private Servo wrist = null;

    // Define constants for wrist servo positions
    private static final double WRIST_COLLECT = 1.0;  // Position for collecting
    private static final double WRIST_DEPOSIT = 0.0; // Position for depositing
    private static final double WRIST_HOME = 0.4;    // Home Position

    public WristCode(Servo wrist) {
        this.wrist = wrist;
        // Ensure the wrist starts at the WRIST_COLLECT position
        wrist.setPosition(WRIST_COLLECT);
    }

    /**
     * Controls the wrist servo based on the dpad buttons pressed.
     *
     * @param leftDpad The state of the left dpad button on the gamepad
     * @param rightDpad The state of the right dpad button on the gamepad
     */

    public void controlWrist(boolean leftDpad, boolean rightDpad) {

        if (leftDpad) {
            wrist.setPosition(WRIST_COLLECT); // Set position to collect when left dpad is pressed
        }
        else if (rightDpad) {
            wrist.setPosition(WRIST_DEPOSIT); // Set position to deposit when right dpad is pressed
        }
        else {
            wrist.setPosition(WRIST_HOME); // Set wrist to home position when no buttons are pressed
        }
    }

    /**
     * Stops the wrist by setting it to the home position.
     */
    public void stopWrist() {
        wrist.setPosition(WRIST_HOME);
    }
}
