package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Utils {

    public static final double SKYSTONE_LENGTH_INCHES = 8;
    public static final double SKYSTONE_WIDTH_INCHES = 4;
    public static final double MAT_LENGTH_INCHES = 24;
    //TODO update the game constants

    private Utils() {
    }

    public static boolean servoNearPosition(Servo servo, double position, double errorTolerance) {
        errorTolerance = Math.abs(errorTolerance);
        return Math.abs(servo.getPosition() - position) <= errorTolerance;
    }

    public static boolean motorNearTarget(DcMotor motor, int errorTolerance) {
        int current = motor.getCurrentPosition();
        int target = motor.getTargetPosition();
        int distance = Math.abs(target - current);
        return distance <= errorTolerance;
    }

    /**
     * Linear interpolation.
     */
    public static double lerp(double min, double max, double interpolant) {
        if (interpolant < 0 || interpolant > 1) {
            throw new IllegalArgumentException("Interpolant must be between 0 and 1 (was " + interpolant + ")");
        }
        return min + (max - min) * interpolant;
    }

    /**
     * Returns an angle in [-pi, pi) that is equal to the specified angle.
     */
    public static double wrapAngle(double radians) {
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        while (radians >= Math.PI) {
            radians -= 2 * Math.PI;
        }
        return radians;
    }

    public double centimetersToInches(double centimeters) {
        return centimeters / 2.54;
    }

    public double inchesToCentimeters(double inches) {
        return inches * 2.54;
    }

    public static void sleep(long milliseconds) {
        try {
            Thread.currentThread().sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
