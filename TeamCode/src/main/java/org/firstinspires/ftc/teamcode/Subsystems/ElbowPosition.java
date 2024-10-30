package org.firstinspires.ftc.teamcode.Subsystems;

public enum ElbowPosition {

    // Constants that store the degrees of the elbow Servo
    GROUND(0),
    HIGH(180),
    FLEX(90);

    // Stores ticks in var value
    private final int value;

    // Assigns the ticks to each constant
    ElbowPosition(int value) {
        this.value = value;
    }

    // Gets the ticks from the enum
    public int getValue() {
        return value;
    }

}
