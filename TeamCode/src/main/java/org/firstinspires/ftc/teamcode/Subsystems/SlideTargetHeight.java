package org.firstinspires.ftc.teamcode.Subsystems;

public enum SlideTargetHeight {

    // Constants that store the ticks for the linear slide levels
    SAMPLE_LOW(1),
    SAMPLE_MEDIUM(2),
    SAMPLE_HIGH(3);

    // Stores ticks in var value
    private final int value;

    // Assigns the ticks to each constant
    SlideTargetHeight(int value) {
        this.value = value;
    }

    // Gets the ticks from the enum
    public int getValue() {
        return value;
    }

}
