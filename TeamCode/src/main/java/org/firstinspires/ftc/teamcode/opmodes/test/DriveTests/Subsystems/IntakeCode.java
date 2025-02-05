package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeCode {

    // Define the CRServo for the intake system
    private CRServo intake = null;

    // Store the current power level of the intake
    public double currentPower = 0.0;

    // Define constants for intake power values
    private static final double INTAKE_COLLECT = 1.0;  // Power for collecting
    private static final double INTAKE_DEPOSIT = -1.0; // Power for depositing
    private static final double INTAKE_OFF = 0.0;      // Power for stopping

    public IntakeCode(CRServo intake) {
        this.intake = intake;
        // Ensure the servo is off at the start
        stopIntake();
    }

    /**
     * Controls the intake servo based on button inputs.
     *
     * @param collectButton   True if the collect button is pressed
     * @param depositButton   True if the deposit button is pressed
     */
    public void controlIntake(boolean collectButton, boolean depositButton) {
        if (collectButton) {
            currentPower = INTAKE_COLLECT; // Set power to collect
        } else if (depositButton) {
            currentPower = INTAKE_DEPOSIT; // Set power to deposit
        } else {
            currentPower = INTAKE_OFF; // Stop the intake
        }

        // Set the intake power
        if (intake != null) {
            intake.setPower(currentPower);
        }
    }

    /**
     * Stops the intake by setting power to zero.
     */
    public void stopIntake() {
        currentPower = INTAKE_OFF;
        if (intake != null) {
            intake.setPower(currentPower);
        }
    }
}
