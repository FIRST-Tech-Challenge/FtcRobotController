package org.firstinspires.ftc.teamcode.Subsystems;

public enum ClawState {

    // Constants that store the values of the claw pos when open and closed
    OPEN(0),
    CLOSE(1); // Change these to reflect irl

    // Stores the pos in var value
    private final int value;

    // Assigns the pos to each constant
    ClawState(int value) {
        this.value = value;
    }

    // Gets the pos from the enum
    public int getValue() {
        return value;
    }

}
