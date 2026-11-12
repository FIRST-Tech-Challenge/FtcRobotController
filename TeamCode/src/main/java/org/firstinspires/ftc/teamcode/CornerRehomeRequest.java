package org.firstinspires.ftc.teamcode;

/**
 * Competition TeleOp GP2 D-pad Left corner placement. The corner pose is written only to a settled READY
 * Pinpoint ({@link PinpointOdometry#isSettled}), the same rule as {@link AutoPoseHandoff} and
 * {@link PedroDrive#setFieldPose}: TeleOp INIT recalibrates the Pinpoint IMU, and a pose written during that
 * reset can be lost while the software already treats it as an explicit field reference.
 *
 * <p>During INIT the robot is stationary, so an early press waits for the Pinpoint to settle. From START on the
 * robot may be moving, so a waiting press is discarded and an unsettled press is refused. No hardware access.
 */
public final class CornerRehomeRequest {
    public static final String NOT_REQUESTED = "Not requested";
    public static final String APPLIED = "APPLIED";
    private boolean waiting, attention;
    private String status = NOT_REQUESTED;

    /**
     * Call on the button's rising edge. Returns true when the corner pose must be applied now.
     *
     * @param pinpointSettled {@link PinpointOdometry#isSettled} from the last Pinpoint read
     * @param mayWait true during INIT (robot stationary): an unsettled press waits instead of being refused
     */
    public boolean press(boolean pinpointSettled, boolean mayWait) {
        if (pinpointSettled) return apply();
        waiting = mayWait;
        attention = true;
        status = mayWait ? "WAITING for Pinpoint READY; keep robot still"
                : "REFUSED: Pinpoint not READY; press again once READY";
        return false;
    }

    /** Call every INIT loop: true exactly once, when a waiting press may be applied. */
    public boolean poll(boolean pinpointSettled) {
        return waiting && pinpointSettled && apply();
    }

    /** Discards a waiting press (START: the robot may move, so the press is no longer known to be valid). */
    public void cancel(String reason) {
        if (!waiting) return;
        waiting = false;
        attention = true;
        status = reason;
    }

    private boolean apply() {
        waiting = attention = false;
        status = APPLIED;
        return true;
    }

    public boolean waiting() { return waiting; }
    /** A press is waiting, was refused or was discarded: the driver should see why the corner was not set. */
    public boolean needsAttention() { return attention; }
    public String status() { return status; }
}
