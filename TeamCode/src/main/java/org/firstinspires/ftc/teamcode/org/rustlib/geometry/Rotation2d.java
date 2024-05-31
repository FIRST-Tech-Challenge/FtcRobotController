package org.firstinspires.ftc.teamcode.org.rustlib.geometry;

public class Rotation2d {

    /**
     * Constructs a new rotation2d with the provided radian value
     **/
    private final double angle;
    private final double cos;
    private final double sin;

    public Rotation2d(double angle) {
        this.angle = signed_minusPI_to_PI(angle);
        cos = Math.cos(angle);
        sin = Math.sin(angle);
    }

    public Rotation2d() {
        this(0);
    }

    public static Rotation2d fromDegrees(double angleDegrees) {
        return new Rotation2d(angleDegrees / 180 * Math.PI);
    }

    public double getAngleRadians() {
        return angle;
    }

    public double getAngleDegrees() {
        return angle * 180 / Math.PI;
    }

    public static double unsigned_0_to_2PI(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public static double signed_minusPI_to_PI(double angle) {
        angle = angle % (2 * Math.PI);
        if (Math.abs(angle) > Math.PI) {
            angle += (2 * Math.PI) * -1 * angle / Math.abs(angle);
        }
        return angle;
    }

    public static double unsigned_0_to_360(double angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public static double signed_minus180_to_180(double angle) {
        angle = angle % 360;
        if (Math.abs(angle) > 180) {
            angle += 360 * -1 * angle / Math.abs(angle);
        }
        return angle;
    }

    public static double minimumMagnitude(double... values) {
        double min = Double.POSITIVE_INFINITY;
        for (double value : values) {
            if (Math.abs(value) < Math.abs(min)) min = value;
        }
        return min;
    }

    public static double getError(double targetAngle, double currentAngle) {
        targetAngle = unsigned_0_to_2PI(targetAngle);
        currentAngle = unsigned_0_to_2PI(currentAngle);
        return minimumMagnitude(targetAngle - currentAngle, targetAngle + 2 * Math.PI - currentAngle, targetAngle - 2 * Math.PI - currentAngle);
    }

    public static Rotation2d averageRotations(Rotation2d[] rotations) {
        if (rotations.length < 1) {
            throw new IllegalArgumentException("Rotation2d array must have at least one item!");
        }
        double x = 0;
        double y = 0;
        for (Rotation2d rotation : rotations) {
            x += rotation.cos;
            y += rotation.sin;
        }
        return new Rotation2d(Math.atan2(y, x));
    }

    public Rotation2d add(Rotation2d rotation) {
        return addRadians(rotation.getAngleRadians());
    }

    public Rotation2d addRadians(double angleRadians) {
        return new Rotation2d(angle + angleRadians);
    }

    public Rotation2d addDegrees(double angleDegrees) {
        return new Rotation2d(angle + Math.toRadians(angleDegrees));
    }

    public Rotation2d negate() {
        return new Rotation2d(-angle);
    }

    @Override
    public String toString() {
        return "angle(" + angle + ")";
    }

    public static String getString(Rotation2d rotation2d) {
        if (rotation2d == null) {
            return "null";
        }
        return rotation2d.toString();
    }
}
