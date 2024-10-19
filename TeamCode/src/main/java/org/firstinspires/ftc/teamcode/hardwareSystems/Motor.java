package org.firstinspires.ftc.teamcode.hardwareSystems;


public enum Motor {
    TETRIX_TORQUENADO(1440);

    private final int TICKS_PER_ROTATION;

    Motor(int ticksPerRotation) {
        this.TICKS_PER_ROTATION = ticksPerRotation;
    }

    public int getTicksPerRotation() {
        return  TICKS_PER_ROTATION;
    }
}