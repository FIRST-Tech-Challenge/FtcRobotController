package org.firstinspires.ftc.teamcode.subsystems;

/*
 * Shot Table
 * Answers one question: "How fast should the FLYWHEEL spin for a shot from here?"
 * The "here" is the Limelight's ty: how far UP the hive's AprilTags appear in the camera view.
 *
 * WHY ty AND NOT DISTANCE
 *   The camera is bolted to the turret, so it always looks the same way. Tags that are CLOSER
 *   appear HIGHER in the view, which means a BIGGER ty. Tags that are FARTHER appear lower, which
 *   means a SMALLER ty. So ty already tells us how far away we are, and we do not need the camera
 *   height or the camera tilt angle. (If the camera is ever moved or re-tilted, the ty values
 *   change, so measure the table again.)
 *
 * HOW IT WORKS
 *   The table below is a list of spots that students measured on the real field. Each spot has:
 *     - the ty the camera showed there, and
 *     - the flywheel RPM that scored from there.
 *   For a ty between two spots, we draw a straight line between them and read off the RPM
 *   (that is called linear interpolation). For a ty OUTSIDE the table we use the nearest spot.
 *   We never guess past the edges, because a wild RPM from a bad reading is worse than a
 *   slightly wrong one.
 *
 * THE HOOD WHEEL
 *   This table has only ONE RPM per spot, for the flywheel. The hood wheel runs at the flywheel
 *   RPM x HOOD_WHEEL_RATIO (see ShooterSubsystem). Pass the answer to
 *   ShooterSubsystem.setTargetRpm(), which does that multiplication for you.
 *   ShooterSubsystem cuts the flywheel target at FLYWHEEL_MAX_TARGET_RPM (1458 RPM, because the
 *   flywheel is the goBILDA 1620 RPM motor). So do not put a flywheel RPM above 1458 in this
 *   table. The farthest spot (1425) is already close to it. (The hood wheel's own limit is far
 *   away at a ratio of 2.0, so the flywheel is the one to watch.)
 *
 * HOW TO EDIT IT (after a ShotTableAlpha session)
 *   1. Type each measured spot into the two lists below. Keep them in the SAME ORDER, and keep
 *      the ty list going from SMALLEST to BIGGEST (that is, from far away to close up).
 *   2. Both lists must be the same length. The first row of one list goes with the first
 *      row of the other.
 *   3. Redeploy and check a few spots with ShotTableAlpha's Left Trigger.
 *
 * This class owns no hardware and never reads a gamepad. Nothing needs update() either:
 * ask it a question and it answers.
 */
public class ShotTable {

    // ------------------------------------------------------------------
    // The table. Each row is one spot on the field.
    // ------------------------------------------------------------------
    // TODO (issue #4): FIRST PASS, measured with ShotTableAlpha (7 saves from 5 different spots, the
    // repeats were merged). The RPMs were found in 50 RPM dial steps, so each is only good to
    // about +/- 50. Row 0 (the farthest spot) is a judgment call between two dial settings:
    // 1400 made the shot and 1450 felt more comfortable, so it is 1425, in between.
    // ty is NEGATIVE here (the tags appear BELOW the middle of the camera view).
    // That is fine. What matters is that a bigger ty (closer to zero) means closer to the hive,
    // and the closer spots needed a LOWER RPM.
    // The closest spot measured is ty -4.0 and the farthest is ty -16.8. Outside that, the table
    // uses the nearest row, so add rows if the robot will shoot from farther or closer.
    // The biggest gaps between measured spots are at the close end (ty -13.1 to -8.7 to -4.0).
    // If shots between those rows miss, measure another spot there.
    //                                           row 0    row 1    row 2    row 3    row 4
    private static final double[] TY_DEGREES =   { -16.8,   -15.8,   -13.1,    -8.7,    -4.0 };
    private static final double[] FLYWHEEL_RPM = { 1425.0,  1350.0,  1300.0,  1250.0,  1250.0 };

    // ==================================================================
    // SETUP
    // ==================================================================

    /*
     * Checks the table when the OpMode starts. A typo here would give strange shot speeds at a
     * match, so we stop right away with a message that says what to fix.
     */
    public ShotTable() {
        if (TY_DEGREES.length != FLYWHEEL_RPM.length) {
            throw new IllegalArgumentException(
                    "ShotTable: TY_DEGREES and FLYWHEEL_RPM must have the same number of rows.");
        }
        if (TY_DEGREES.length == 0) {
            throw new IllegalArgumentException("ShotTable: the table needs at least one row.");
        }
        for (int row = 1; row < TY_DEGREES.length; row = row + 1) {
            if (TY_DEGREES[row] <= TY_DEGREES[row - 1]) {
                throw new IllegalArgumentException(
                        "ShotTable: TY_DEGREES must go from SMALLEST to BIGGEST with no repeats. "
                                + "Check row " + row + ".");
            }
        }
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks about a shot
    // ==================================================================

    /*
     * The flywheel RPM for a shot from where the camera reads this ty.
     * Between two rows it draws a straight line. Outside the table it uses the nearest row.
     */
    public double getFlywheelRpm(double tyDegrees) {
        int lastRow = TY_DEGREES.length - 1;

        // Below the first row or above the last row: use the nearest row.
        if (tyDegrees <= TY_DEGREES[0]) {
            return FLYWHEEL_RPM[0];
        }
        if (tyDegrees >= TY_DEGREES[lastRow]) {
            return FLYWHEEL_RPM[lastRow];
        }

        // Find the two rows that this ty sits between.
        for (int row = 0; row < lastRow; row = row + 1) {
            double lowTy = TY_DEGREES[row];
            double highTy = TY_DEGREES[row + 1];
            if (tyDegrees <= highTy) {
                // fraction is how far we are from the low row to the high row:
                // 0.0 means right on the low row, 1.0 means right on the high row.
                double fraction = (tyDegrees - lowTy) / (highTy - lowTy);
                double lowRpm = FLYWHEEL_RPM[row];
                double highRpm = FLYWHEEL_RPM[row + 1];
                return lowRpm + fraction * (highRpm - lowRpm);
            }
        }

        // We can only get here if the loop above missed. The checks above make that impossible,
        // but Java needs a return, and the last row is the safest answer.
        return FLYWHEEL_RPM[lastRow];
    }

    /*
     * True when this ty is inside the spots we measured (between the first and last row).
     * Outside that range getFlywheelRpm() still answers, but with the nearest row's RPM,
     * so show a warning.
     */
    public boolean isInRange(double tyDegrees) {
        int lastRow = TY_DEGREES.length - 1;
        return tyDegrees >= TY_DEGREES[0] && tyDegrees <= TY_DEGREES[lastRow];
    }

    // How many rows the table has. The test OpMode shows this on the screen.
    public int getRowCount() {
        return TY_DEGREES.length;
    }
}
