package org.firstinspires.ftc.teamcode;

/**
 * Autonomous INIT relocalization from a known start pose. Hardware-free; {@link BioBuzzRobot} wires it.
 *
 * <p>Every routine starts at an authored pose, so odometry is seeded with that pose first. The turret then
 * points the Limelight at a hive cell that is raised at the start of every match, and the existing corner
 * refinement ({@link FieldRehomeRefinement}: 5 consistent frames, at most 0.15 m, translation only) tries to
 * correct it. On success the refined pose is written back into Pinpoint once, so Pedro paths and aiming
 * share it. Otherwise the start pose stays and the refinement is retried until START.
 */
public final class AutoStartRelocalization {
    public enum State { WAITING, REFINING, REFINED, START_POSE_ONLY }
    public enum Decision { NONE, REQUEST_REFINEMENT, APPLY_REFINED_POSE }

    /** Substring of the refinement's per-tag rejection when tags disagree with the seeded pose beyond its bound. */
    static final String OUT_OF_BOUND_REASON = "exceeds corner bound";
    static final String REFINED_PREFIX = "VISION REFINED";

    private State state = State.WAITING;
    private String status = "WAITING: turret homing / Pinpoint start pose";
    private boolean requested, outOfBoundThisAttempt, checkPlacement;
    private int attempts;
    private double correctionInches = Double.NaN;

    /**
     * Cells raised at the start of every match (Event Field Setup Guide 11.1): red audience and blue far.
     * The landmark is the one on the robot's half of the field, whichever alliance it is. Red's far cell and
     * blue's audience cell start lowered and cannot localize.
     */
    public static GoalTarget landmarkFor(double fieldYMeters) {
        return fieldYMeters < 0
                ? TeleOpFieldReference.target(TurretSubsystem.Alliance.RED, TurretSubsystem.HiveSide.AUDIENCE)
                : TeleOpFieldReference.target(TurretSubsystem.Alliance.BLUE, TurretSubsystem.HiveSide.FAR);
    }

    /**
     * Call once per INIT loop after the shot coordinator has processed this loop's pose and camera frame.
     *
     * @param homed             turret homed (camera bearing known)
     * @param poseReady         Pinpoint READY with the seeded, field-referenced start pose
     * @param refinementPending the coordinator's refinement is still collecting frames
     * @param refinementStatus  the coordinator's refinement status text
     */
    public Decision update(boolean homed, boolean poseReady, boolean refinementPending, String refinementStatus) {
        if (state == State.REFINED) return Decision.NONE;
        if (!homed || !poseReady) {
            if (!requested) status = "WAITING: " + (!homed ? "turret homing" : "Pinpoint start pose");
            return Decision.NONE;
        }
        if (!requested) {
            requested = true; outOfBoundThisAttempt = false; attempts++;
            if (state == State.WAITING) state = State.REFINING;
            return Decision.REQUEST_REFINEMENT;
        }
        String text = refinementStatus == null ? "" : refinementStatus;
        if (refinementPending) {
            if (text.contains(OUT_OF_BOUND_REASON)) outOfBoundThisAttempt = true;
            if (state == State.REFINING) status = "REFINING (attempt " + attempts + "): " + text;
            return Decision.NONE;
        }
        requested = false;
        if (text.startsWith(REFINED_PREFIX)) {
            state = State.REFINED; checkPlacement = false;
            status = "REFINED from hive tags";
            return Decision.APPLY_REFINED_POSE;
        }
        state = State.START_POSE_ONLY;
        checkPlacement = outOfBoundThisAttempt;
        status = "START POSE ONLY (attempt " + attempts + "): " + text + "; retrying";
        return Decision.NONE;
    }

    /** Record how far the written-back refined pose moved from the seeded start pose. */
    public void applied(double movedInches) {
        correctionInches = movedInches;
        status = String.format(java.util.Locale.US, "REFINED from hive tags (moved %.1f in)", movedInches);
    }

    public State state() { return state; }
    public String status() { return status; }
    public int attempts() { return attempts; }
    public double correctionInches() { return correctionInches; }
    /** The last attempt saw tags that disagree with the start pose by more than the refinement bound. */
    public boolean checkPlacement() { return checkPlacement; }
}
