package org.firstinspires.ftc.teamcode.Subsystems;

public enum GripperMode {

    // Constants that store the ticks for the linear slide levels
    INTAKE (1),
    EJECT (0),
    OFF (0.5);

    // Store
    private final double value;

    // Assigns the ticks to each constant
    GripperMode(double value) {
        this.value = value;
    }

    // Gets the ticks from the enum
    public double getValue() {
        return value;
    }

}
