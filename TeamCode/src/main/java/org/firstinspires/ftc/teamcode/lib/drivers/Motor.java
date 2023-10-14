package org.firstinspires.ftc.teamcode.lib.drivers;

public enum Motor {
    GOBILDA_435_RPM(435d, 383.6d),
    GOBILDA_312_RPM(312d, 537.6d),
    GOBILDA_223_RPM(223d, 753.2d),
    NEVERST_3_7(1780d, 103.6d),
    REV_CORE_HEX(125d, 288d);

    private final double RPM;
    private final double ENCODER_TICKS_PER_REVOLUTION;

    Motor(double RPM, double encoderTicksPerRevolution) {
        this.RPM = RPM;
        this.ENCODER_TICKS_PER_REVOLUTION = encoderTicksPerRevolution;
    }

    public double maxAngularVelocity() {
        return getRPM() * Math.PI / 30d;
    }

    public double getRPM() {
        return RPM;
    }

    public double getENCODER_TICKS_PER_REVOLUTION() {
        return ENCODER_TICKS_PER_REVOLUTION;
    }
}
