package org.firstinspires.ftc.teamcode.common;

public class Angle {
    // Angle normalized to [0, 360) degrees.
    private double degrees;
    private Angle(double degrees) {
        this.degrees = degrees;
        normalize();
    }
    public static Angle fromEncoderTicks(int encoderTicks) {
        return new Angle(encoderTicks * Config.ROBOT_DEGREES_PER_TICK);
    }
    public static Angle fromRotations(double rotations) {
        return new Angle(rotations * 360d);
    }
    public static Angle fromDegrees(double degrees) {
        return new Angle(degrees);
    }

    public double toRotations() {
        return degrees / 360d;
    }

    public double toDegrees() {
        return degrees;
    }
    public double toRadians() { return getRadians(); }

    public int toEncoderTicks() {
        return (int) (degrees / Config.ROBOT_DEGREES_PER_TICK);
    }
    public double getRotations() { return toRotations(); }
    public double getDegrees() { return toDegrees(); }
    public long getEncoderTicks() { return toEncoderTicks(); }
    public Angle normalize() {
        degrees += 180;
        degrees %= 360;
        degrees -= 180;
        return this;
    }
    public Angle add(Angle addend) {
        degrees += addend.degrees;
        return this.normalize();
    }
    public Angle divide(double dividend) {
        degrees /= dividend;
        return this.normalize();
    }
    public Angle subtract(Angle subtractand) {
        degrees -= subtractand.degrees;
        return this.normalize();
    }
    public Angle multiply(double multiplicand) {
        degrees *= multiplicand;
        return this.normalize();
    }

    public static Angle aTan(double x, double y) {
        double a = 0;
        if (y == 0) {
            if (x < 0) {
                a = -90;
            } else if (x > 0) {
                a = 90;
            } else if (x == 0) {
                a = 0;
            }
        } else if (x == 0) {
            a = 0;
        } else {
            a = (Math.atan(x/y)*(180/Math.PI));
        }

        if (y < 0) {
            if (x < 0) {
                a -= 180;
            } else {
                a += 180;
            }
        }
        return Angle.fromDegrees(a);
    }

    public Angle copy() {
        return Angle.fromDegrees(degrees);
    }

    public double getRadians() {
        return getDegrees() * Math.PI / 180d;
    }
}