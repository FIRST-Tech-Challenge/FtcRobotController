package org.firstinspires.ftc.teamcode.Subsystems;

public enum SlideTargetHeight {

    // Constants that store the ticks for the linear slide levels
    SAMPLE_ZERO(0), // 0 ticks = 150mm up = min based on the mid bar
    SAMLE_SPECIMEN(40),// 0 ticks = 312mm
    SAMPLE_LOW(750), // 750 ticks = 362mm
    SAMPLE_MEDIUM(1500), // 1500 ticks = 575mm
    SAMPLE_HIGH(2300);// 2630 ticks = 880mm = max


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
