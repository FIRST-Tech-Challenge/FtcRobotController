package org.firstinspires.ftc.teamcode.Subsystems;

public enum ClimbTargetHeight {

    // Constants that store the ticks for the linear slide levels

    SAMPLE_CLIMB_ZERO(0),
    SAMPLE_LIFT(2381);




    // Stores ticks in var value
    private final int value;

    // Assigns the ticks to each constant
    ClimbTargetHeight(int value) {
        this.value = value;
    }

    // Gets the ticks from the enum
    public int getValue() {
        return value;
    }

}
