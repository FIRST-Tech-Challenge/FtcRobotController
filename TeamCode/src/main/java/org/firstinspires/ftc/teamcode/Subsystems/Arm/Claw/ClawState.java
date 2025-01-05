package org.firstinspires.ftc.teamcode.Subsystems.Arm.Claw;

public enum ClawState {

    // Constants that store the values of the claw pos when open and closed
    OPEN(0.4),
    CLOSE(0.0); // Change these to reflect irl

    // Stores the pos in var value
    private final double value;

    // Assigns the pos to each constant
    ClawState(double value) {
        this.value = value;
    }

    // Gets the pos from the enum
    public double getValue() {
        return value;
    }

}
