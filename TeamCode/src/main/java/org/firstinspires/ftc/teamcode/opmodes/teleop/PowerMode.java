package org.firstinspires.ftc.teamcode.opmodes.teleop;

public enum PowerMode {
    REGULAR(.8),
    NITRO(1),
    SLOW(.3);

    PowerMode(double powerRatio) {
        this.powerRatio = powerRatio;
    }

    public double getPowerRatio() {
        return powerRatio;
    }

    private final double powerRatio;
}
