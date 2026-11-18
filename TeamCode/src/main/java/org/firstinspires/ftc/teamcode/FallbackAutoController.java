package org.firstinspires.ftc.teamcode;

/**
 * Hardware-free fallback autonomous: an optional preload shot from the start pose, then a short
 * Pinpoint-checked drive out of the start area. It needs no Pedro tuning, no field pose and no shot
 * calibration, so it can run while {@link PedroConfiguration#TUNING_VERIFIED} is false.
 *
 * <p>Distances are relative to the robot's pose when autonomous starts (+forward, +left, inches);
 * the heading at START is held throughout. Pinpoint convention: +X forward, +Y left, CCW heading
 * ({@link RobotMath}). Call {@link #update} once per loop; it never blocks and touches no hardware.
 */
public final class FallbackAutoController {

    public enum Phase { START_DELAY, SPIN_UP, FEED, LEAVE, DONE, FAULT }

    // ---- INIT selection ----
    public static final double MAX_START_DELAY_SECONDS = 15.0;
    public static final double START_DELAY_STEP_SECONDS = 1.0;

    // ---- Preload shot. STARTING VALUES, UNMEASURED: confirm on the practice field. ----
    // Feed anyway after this long if the turret is homed but RPM / hood / yaw never report ready.
    public static final double SPIN_UP_TIMEOUT_SECONDS = 3.0;
    // Continuous feed time for every preload to leave (both transfer stages, gate open).
    public static final double FEED_SECONDS = 2.5;

    // ---- Leave drive. STARTING VALUES: deliberately slow, same 0.35 cap as Pedro autonomous. ----
    public static final double KP_POWER_PER_INCH = 0.04;       // 25 in of error requests 1.0 (then capped)
    public static final double MAX_DRIVE_POWER = 0.35;
    public static final double MIN_DRIVE_POWER = 0.08;         // Overcomes static friction near the target
    public static final double KP_YAW_POWER_PER_DEG = 0.01;
    public static final double MAX_YAW_POWER = 0.25;
    public static final double POSITION_TOLERANCE_INCHES = 1.0;
    public static final double HEADING_TOLERANCE_DEG = 3.0;
    public static final double LEAVE_TIMEOUT_SECONDS = 4.0;
    // Getting this much farther from the target than at the start means a drive/odometry sign is wrong.
    public static final double WRONG_WAY_INCHES = 6.0;
    public static final double POSE_MAX_AGE_SECONDS = 0.1;
    public static final double POSE_LOSS_SECONDS = 0.25;
    public static final double MAX_AUTONOMOUS_SECONDS = PedroConfiguration.MAX_AUTONOMOUS_SECONDS;

    private static final double INCHES_PER_METER = 1 / .0254;

    private final double startDelaySeconds, leaveForwardInches, leaveLeftInches;
    private final boolean shootPreloads;

    private Phase phase = Phase.START_DELAY;
    private double started = Double.NaN, phaseStarted = Double.NaN, poseLostSince = Double.NaN;
    private boolean hasOrigin;
    private double originXInches, originYInches, originHeading;
    private double leaveStartDistanceInches = Double.NaN, remainingInches = Double.NaN;
    private boolean fedBeforeReady;
    private String status = "Waiting for START";

    private double axial, lateral, yaw;
    private boolean flywheelOn, feedRequested;

    public FallbackAutoController(double startDelaySeconds, boolean shootPreloads,
                                  double leaveForwardInches, double leaveLeftInches) {
        if (!Double.isFinite(startDelaySeconds) || startDelaySeconds < 0 || startDelaySeconds > MAX_START_DELAY_SECONDS
                || !RobotMath.finite(leaveForwardInches, leaveLeftInches))
            throw new IllegalArgumentException("Invalid fallback autonomous settings");
        this.startDelaySeconds = startDelaySeconds;
        this.shootPreloads = shootPreloads;
        this.leaveForwardInches = leaveForwardInches;
        this.leaveLeftInches = leaveLeftInches;
    }

    /** INIT helper: steps the delay and keeps it within 0..{@link #MAX_START_DELAY_SECONDS}. */
    public static double stepDelay(double delaySeconds, int steps) {
        return Math.max(0, Math.min(MAX_START_DELAY_SECONDS, delaySeconds + steps * START_DELAY_STEP_SECONDS));
    }

    /** Call once at START (System.nanoTime clock, seconds). */
    public void start(double now) {
        started = now;
        enter(Phase.START_DELAY, now, shootPreloads ? "Start delay (flywheel spinning up)" : "Start delay");
    }

    /**
     * @param pose          this loop's Pinpoint sample (may be null)
     * @param turretHomed   turret has a valid home, so a yaw command means something
     * @param shooterReady  flywheel at speed, hood settled and yaw on target
     */
    public void update(double now, MotionSample pose, boolean turretHomed, boolean shooterReady) {
        axial = lateral = yaw = 0;
        flywheelOn = feedRequested = false;
        if (Double.isNaN(started) || phase == Phase.DONE || phase == Phase.FAULT) return;
        if (now - started >= MAX_AUTONOMOUS_SECONDS) { enter(Phase.DONE, now, "Stopped at autonomous deadline"); return; }

        boolean poseValid = valid(pose, now);
        if (!hasOrigin && poseValid) {
            originXInches = pose.x * INCHES_PER_METER;
            originYInches = pose.y * INCHES_PER_METER;
            originHeading = pose.heading;
            hasOrigin = true;
        }

        switch (phase) {
            case START_DELAY:
                flywheelOn = shootPreloads;
                if (now - phaseStarted >= startDelaySeconds) {
                    if (shootPreloads) enter(Phase.SPIN_UP, now, "Waiting for flywheel / hood / turret");
                    else enter(Phase.LEAVE, now, "Leaving start area");
                }
                break;
            case SPIN_UP:
                flywheelOn = true;
                if (turretHomed && shooterReady) {
                    enter(Phase.FEED, now, "Feeding preloads");
                    feedRequested = true;
                } else if (now - phaseStarted >= SPIN_UP_TIMEOUT_SECONDS) {
                    if (turretHomed) {
                        fedBeforeReady = true;
                        enter(Phase.FEED, now, "Feeding preloads (spin-up timed out before READY)");
                        feedRequested = true;
                    } else {
                        flywheelOn = false;
                        enter(Phase.LEAVE, now, "Shot SKIPPED: turret not homed");
                    }
                }
                break;
            case FEED:
                if (now - phaseStarted >= FEED_SECONDS) {
                    enter(Phase.LEAVE, now, "Leaving start area");
                } else {
                    flywheelOn = true;
                    feedRequested = true;
                }
                break;
            case LEAVE:
                leave(now, pose, poseValid);
                break;
            default:
                break;
        }
    }

    private void leave(double now, MotionSample pose, boolean poseValid) {
        if (!poseValid || !hasOrigin) {
            if (Double.isNaN(poseLostSince)) poseLostSince = now;
            if (now - poseLostSince >= POSE_LOSS_SECONDS) enter(Phase.FAULT, now, "Pinpoint pose unavailable / stale; stopped");
            else status = "Waiting for Pinpoint pose";
            return;
        }
        poseLostSince = Double.NaN;

        double cos0 = Math.cos(originHeading), sin0 = Math.sin(originHeading);
        double targetX = originXInches + leaveForwardInches * cos0 - leaveLeftInches * sin0;
        double targetY = originYInches + leaveForwardInches * sin0 + leaveLeftInches * cos0;
        double ex = targetX - pose.x * INCHES_PER_METER, ey = targetY - pose.y * INCHES_PER_METER;
        double distance = Math.hypot(ex, ey);
        double headingErrorDeg = Math.toDegrees(RobotMath.wrap(originHeading - pose.heading));
        remainingInches = distance;
        if (Double.isNaN(leaveStartDistanceInches)) leaveStartDistanceInches = distance;

        if (distance > leaveStartDistanceInches + WRONG_WAY_INCHES) {
            enter(Phase.FAULT, now, "Moved AWAY from target: check drive / Pinpoint directions");
            return;
        }
        if (distance <= POSITION_TOLERANCE_INCHES && Math.abs(headingErrorDeg) <= HEADING_TOLERANCE_DEG) {
            enter(Phase.DONE, now, "Leave complete");
            return;
        }
        if (now - phaseStarted >= LEAVE_TIMEOUT_SECONDS) {
            enter(Phase.DONE, now, String.format(java.util.Locale.US, "Leave TIMEOUT, %.1f in short", distance));
            return;
        }

        if (distance > POSITION_TOLERANCE_INCHES) {
            // Field-frame error into the robot frame at the current heading.
            double cos = Math.cos(pose.heading), sin = Math.sin(pose.heading);
            double forward = ex * cos + ey * sin;
            double left = -ex * sin + ey * cos;
            double power = Math.max(MIN_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, KP_POWER_PER_INCH * distance));
            axial = power * forward / distance;
            lateral = -power * left / distance; // MecanumDrivetrain: +lateral = right
        }
        // MecanumDrivetrain: +yaw = clockwise, so a CCW (positive) heading error needs negative yaw.
        yaw = RobotMath.clip(-KP_YAW_POWER_PER_DEG * headingErrorDeg, MAX_YAW_POWER);
        status = "Leaving start area";
    }

    private static boolean valid(MotionSample pose, double now) {
        return pose != null && pose.ready && RobotMath.finite(pose.time, pose.x, pose.y, pose.heading)
                && now >= pose.time && now - pose.time <= POSE_MAX_AGE_SECONDS;
    }

    private void enter(Phase next, double now, String reason) {
        phase = next; phaseStarted = now; status = reason;
    }

    public Phase phase() { return phase; }
    public String status() { return status; }
    public boolean shootPreloads() { return shootPreloads; }
    public double startDelaySeconds() { return startDelaySeconds; }
    public boolean fedBeforeReady() { return fedBeforeReady; }
    /** Inches still to go once LEAVE has a pose; NaN before. */
    public double remainingInches() { return remainingInches; }
    public double elapsedSeconds(double now) { return Double.isNaN(started) ? 0 : now - started; }
    /** Robot-centric drive powers for {@link MecanumDrivetrain#driveRobotCentric}. */
    public double axial() { return axial; }
    public double lateral() { return lateral; }
    public double yaw() { return yaw; }
    public boolean flywheelOn() { return flywheelOn; }
    /** Hold-to-feed request, equivalent to the driver holding manual feed in TeleOp. */
    public boolean feedRequested() { return feedRequested; }
}
