package org.firstinspires.ftc.teamcode.common;

public class Distance {
    private int encoderTicks = 0;
    private Distance(int encoderTicks) {
        this.encoderTicks = encoderTicks;
    }

    public static Distance fromEncoderTicks(int encoderTicks) {
        return new Distance(encoderTicks);
    }

    public static Distance fromDegrees(double degrees) {
        return new Distance((int) (degrees * Config.ENCODER_TICKS_PER_SHAFT_DEGREE));
    }

    public static Distance fromRotations(double rotations) {
        return new Distance((int) (rotations * 360 * Config.ENCODER_TICKS_PER_SHAFT_DEGREE));
    }

    public static Distance fromInches(double inches) {
        return new Distance((int) (inches * Config.ENCODER_TICKS_PER_INCH));
    }

    public static Distance fromMillimeters(double millimeters) {
        return fromInches(millimeters / 25.4);
    }

    public Distance subtract(Distance subtractend) {
        encoderTicks -= subtractend.encoderTicks;
        return this;
    }

    public Distance add(Distance addend) {
        encoderTicks += addend.encoderTicks;
        return this;
    }

    public Distance copy()
    {
        return Distance.fromEncoderTicks(encoderTicks);
    }

    public double toRotations() {
        return encoderTicks / 396000.0d;
    }
    public double toDegrees() {
        return encoderTicks / 360.0d;
    }
    public int toEncoderTicks() {
        return encoderTicks;
    }
    public double getRotations() {
        return toRotations();
    }
    public double getDegrees() {
        return toDegrees();
    }
    public long getEncoderTicks() {
        return toEncoderTicks();
    }
    public double toInches() {
        return encoderTicks / Config.ENCODER_TICKS_PER_INCH;
    }
    public double toMillimeters() {
        return toInches() * 25.4;
    }
}