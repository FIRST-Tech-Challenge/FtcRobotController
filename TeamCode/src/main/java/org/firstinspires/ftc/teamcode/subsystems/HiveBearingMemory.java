package org.firstinspires.ftc.teamcode.subsystems;

/*
 * Hive Bearing Memory
 * A plain class with NO hardware, no update(), and no gamepad code. It is not a mechanism, so its
 * name does not end in "Subsystem" (the same as ShotTable). It does one job: REMEMBER which
 * direction the hive was in the last time the camera saw it, so the turret can keep pointing that
 * way when the camera loses the hive.
 *
 * WHY THE DIRECTION IS "FIELD" DIRECTION
 *   The turret angle is measured from the robot, so when the robot spins, the turret angle has to
 *   change to keep pointing at the same spot. But a direction measured on the FIELD does not
 *   change when the robot spins. The Pinpoint tells us the robot's heading, so:
 *       direction to the hive, on the field = robotHeading + turretAngle - tx
 *       turret angle that points at it      = that direction - robotHeading
 *   (tx is positive to the RIGHT, and every other angle here is positive to the LEFT, so tx is
 *   subtracted, the same flip the auto-align already does.)
 *   The Pinpoint's heading starts at 0 wherever the robot sat at INIT, and that is fine: we only
 *   ever compare headings from the same match, so the starting direction never matters.
 *
 * WHEN THE MEMORY IS NOT TRUSTED (STALE)
 *   Spinning does not change the direction to the hive, but DRIVING does. Driving sideways or
 *   forward moves the robot, so the real direction to the hive changes, and heading alone cannot
 *   see it. So the memory is stale when the robot has driven more than
 *   MAX_DRIVE_SINCE_SEEN_INCHES since the camera last saw the hive (the Pinpoint's X and Y
 *   measure that), or when more than MAX_HOLD_SECONDS have passed.
 *   Spinning in place barely changes X and Y, so a spin never makes the memory stale.
 *
 * HOW THE OPMODE USES IT
 *   Every loop the camera sees the hive:  remember(...)
 *   Every loop it does not:               if hasMemory() and not isStale(...), aim the turret at
 *                                         getTurretAngleDegrees(robotHeading).
 *   When the alliance changes:            clear(), so we never aim at the other hive's direction.
 *
 * ONE LIMIT: the two cells of our hive are on the same frame, only about 19 inches apart, so one
 * remembered direction works for either side of the hive. It cannot tell which side is raised.
 * (VisionSubsystem decides that from the tags.)
 */
public class HiveBearingMemory {

    // ------------------------------------------------------------------
    // When the memory goes stale. TODO (issue #48): Both are starting guesses to tune on the robot.
    // ------------------------------------------------------------------
    // Stale once the robot has driven this many inches (in a straight line) from the spot where
    // the camera last saw the hive. Why 12: the Limelight sees about 27 degrees to each side,
    // and 12 inches of sideways drive at about 5 feet from the hive moves the hive about 11
    // degrees in the view. So after 12 inches the hive is still near the middle of the picture,
    // and the camera can find it again. Raise it if the turret gives up too soon, lower it if the
    // turret points off the hive when it should have given up.
    private static final double MAX_DRIVE_SINCE_SEEN_INCHES = 12.0;

    // Stale once this many seconds pass without the camera seeing the hive. This stops the
    // turret from following an old direction for the rest of the match. The Pinpoint's gyro also
    // drifts a little, so older memories are worth less.
    private static final double MAX_HOLD_SECONDS = 5.0;

    // ------------------------------------------------------------------
    // What we remember
    // ------------------------------------------------------------------
    private boolean hasMemory = false;

    // The direction to the hive on the field, in degrees from -180 to 180 (left is positive).
    private double hiveFieldHeadingDegrees = 0.0;

    // Where the robot was when the camera last saw the hive, in inches (the Pinpoint's X and Y).
    private double seenXInches = 0.0;
    private double seenYInches = 0.0;

    // ==================================================================
    // ACTIONS: the OpMode tells the memory what happened
    // ==================================================================

    /*
     * Call this EVERY loop the camera can see the hive. It saves the direction to the hive and
     * where the robot is standing.
     *   robotHeadingDegrees  -> PinpointSubsystem.getHeadingDegrees()
     *   turretAngleDegrees   -> TurretSubsystem.getAngleDegrees()
     *   txDegrees            -> VisionSubsystem.getTxDegrees() (positive = hive is to the RIGHT)
     *   robotXInches, robotYInches -> PinpointSubsystem.getXInches() and getYInches()
     */
    public void remember(double robotHeadingDegrees, double turretAngleDegrees, double txDegrees,
                         double robotXInches, double robotYInches) {
        hiveFieldHeadingDegrees = wrapDegrees(robotHeadingDegrees + turretAngleDegrees - txDegrees);
        seenXInches = robotXInches;
        seenYInches = robotYInches;
        hasMemory = true;
    }

    /*
     * Forget everything. Call it when the alliance changes, because the remembered direction
     * belongs to the other hive.
     */
    public void clear() {
        hasMemory = false;
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks what we remember
    // ==================================================================

    /*
     * True once the camera has seen the hive at least once since the last clear().
     */
    public boolean hasMemory() {
        return hasMemory;
    }

    /*
     * The turret angle (degrees, left is positive) that points at the remembered hive direction
     * when the robot is facing robotHeadingDegrees. The answer is from -180 to 180. The turret
     * itself still keeps it inside its own soft limits.
     */
    public double getTurretAngleDegrees(double robotHeadingDegrees) {
        return wrapDegrees(hiveFieldHeadingDegrees - robotHeadingDegrees);
    }

    /*
     * How far (in a straight line, in inches) the robot is from where it stood when the camera
     * last saw the hive.
     */
    public double getDrivenInchesSinceSeen(double robotXInches, double robotYInches) {
        double xDifference = robotXInches - seenXInches;
        double yDifference = robotYInches - seenYInches;
        return Math.sqrt(xDifference * xDifference + yDifference * yDifference);
    }

    /*
     * True when the memory should NOT be trusted: the robot has driven too far since the camera
     * last saw the hive, or too many seconds have passed. (Also true when there is no memory.)
     * secondsSinceSeen is how long ago the camera last saw the hive.
     */
    public boolean isStale(double robotXInches, double robotYInches, double secondsSinceSeen) {
        if (!hasMemory) {
            return true;
        }
        if (secondsSinceSeen > MAX_HOLD_SECONDS) {
            return true;
        }
        return getDrivenInchesSinceSeen(robotXInches, robotYInches) > MAX_DRIVE_SINCE_SEEN_INCHES;
    }

    /*
     * The remembered direction to the hive on the field, for the screen. Only means something
     * when hasMemory() is true.
     */
    public double getHiveFieldHeadingDegrees() {
        return hiveFieldHeadingDegrees;
    }

    // The limits, for the screen, so the telemetry never copies the numbers.
    public double getMaxDriveSinceSeenInches() {
        return MAX_DRIVE_SINCE_SEEN_INCHES;
    }

    public double getMaxHoldSeconds() {
        return MAX_HOLD_SECONDS;
    }

    // ==================================================================
    // HELPERS
    // ==================================================================

    /*
     * Turns any angle into the same direction between -180 and 180 degrees. For example 270
     * becomes -90, and -190 becomes 170. (Headings wrap around at the back of the robot, so
     * without this, 179 degrees and -179 degrees would look 358 degrees apart instead of 2.)
     */
    public static double wrapDegrees(double degrees) {
        double wrapped = degrees;
        while (wrapped > 180.0) {
            wrapped = wrapped - 360.0;
        }
        while (wrapped <= -180.0) {
            wrapped = wrapped + 360.0;
        }
        return wrapped;
    }
}
