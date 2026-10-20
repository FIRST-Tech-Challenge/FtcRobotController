package org.firstinspires.ftc.teamcode.objects;

/**
 * The turn that points the robot at something it can see.
 *
 * <p>The further off center the target is, the harder the robot turns, up to a limit.
 * Close enough to center, it stops turning, so it does not wobble back and forth. No
 * hardware here, so it can be tested on a laptop.
 */
public final class AimAssist {
    private AimAssist() {
    }

    /**
     * @param bearingDegrees degrees from straight ahead to the target. Left is positive.
     * @param kP             turn power per degree
     * @param maxRotate      the turn never goes past this, either way
     * @param doneDegrees    within this many degrees of center, stop turning
     * @return the rotate power for the drive. Positive turns left. A broken number (NaN)
     *     turns nothing.
     */
    public static double RotateToward(double bearingDegrees, double kP, double maxRotate,
                                      double doneDegrees) {
        if (Double.isNaN(bearingDegrees)) {
            return 0;
        }
        if (Math.abs(bearingDegrees) <= doneDegrees) {
            return 0;
        }
        double rotate = kP * bearingDegrees;
        return Math.max(-maxRotate, Math.min(rotate, maxRotate));
    }
}
