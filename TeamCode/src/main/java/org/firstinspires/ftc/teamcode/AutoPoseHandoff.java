package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/**
 * Carries the last field-referenced autonomous pose into competition TeleOp, so auto aim works from the
 * first TeleOp loop instead of waiting for a corner rehome or a Limelight acquisition.
 *
 * <p>RAM only, within one Robot Controller app session: an app restart loses it and TeleOp falls back to
 * the camera / corner. A pose is used at most once, only by the same alliance and only shortly after
 * autonomous ended. The robot must not move in between, which match rules guarantee between periods but
 * practice does not: GP2 D-pad Left (corner rehome) always replaces it.
 */
public final class AutoPoseHandoff {
    private AutoPoseHandoff() { }

    /**
     * STARTING VALUE: a match normally reaches TeleOp INIT within about 10-20 s of autonomous ending.
     * Older poses are ignored as likely practice runs where the robot was carried away.
     */
    public static final double MAX_AGE_SECONDS = 60;
    /**
     * TeleOp recalibrates the Pinpoint IMU (about 0.25 s, CALIBRATING) during INIT. The pose is written only
     * after the device has reported READY continuously for this long, so the reset cannot overwrite it.
     */
    public static final double PINPOINT_SETTLE_SECONDS = PinpointOdometry.READY_SETTLE_SECONDS;

    /** Field pose: meters, CCW radians, field-center origin; time on the System.nanoTime clock (s). */
    public static final class Pose {
        public final TurretSubsystem.Alliance alliance;
        public final double x, y, heading, time;
        Pose(TurretSubsystem.Alliance alliance, double x, double y, double heading, double time) {
            this.alliance = alliance; this.x = x; this.y = y; this.heading = heading; this.time = time;
        }
        public Pose2D toPose2D() { return new Pose2D(DistanceUnit.METER, x, y, AngleUnit.RADIANS, heading); }
    }

    private static Pose latest;

    /** Autonomous, every loop: record the latest READY, field-referenced pose. Local or invalid samples are ignored. */
    public static synchronized void save(TurretSubsystem.Alliance alliance, MotionSample sample) {
        if (alliance == null || sample == null || !sample.ready || !sample.fieldReferenced
                || !RobotMath.finite(sample.time, sample.x, sample.y, sample.heading)) return;
        latest = new Pose(alliance, sample.x, sample.y, sample.heading, sample.time);
    }
    /** Autonomous INIT or localization fault: no pose from this run (or an older one) may reach TeleOp. */
    public static synchronized void clear() { latest = null; }

    /** TeleOp INIT: consumes any stored pose (used or not) and returns the INIT-side state for this alliance. */
    public static synchronized Pending take(TurretSubsystem.Alliance alliance, double now) {
        Pose pose = latest;
        latest = null;
        if (pose == null) return new Pending(null, "None: no autonomous pose this session");
        double age = now - pose.time;
        if (pose.alliance != alliance) return new Pending(null, "Ignored: autonomous pose was " + pose.alliance);
        if (!(age >= 0) || age > MAX_AGE_SECONDS)
            return new Pending(null, String.format(Locale.US, "Ignored: autonomous pose %.0f s old (limit %.0f s)",
                    age, MAX_AGE_SECONDS));
        return new Pending(pose, String.format(Locale.US,
                "Waiting for Pinpoint READY (autonomous pose %.0f s old)", age));
    }

    /** TeleOp INIT-side state: applies the pose once, after the Pinpoint has settled. No hardware access. */
    public static final class Pending {
        private Pose pose;
        private double readySince = Double.NaN;
        private String status;
        Pending(Pose pose, String status) { this.pose = pose; this.status = status; }

        /** Call once per INIT loop before the Pinpoint read; returns the pose to set exactly once, else null. */
        public Pose2D poll(double now, boolean pinpointReady) {
            if (pose == null) return null;
            if (!pinpointReady || !Double.isFinite(now)) { readySince = Double.NaN; return null; }
            if (Double.isNaN(readySince)) readySince = now;
            if (now - readySince < PINPOINT_SETTLE_SECONDS) return null;
            Pose applied = pose;
            pose = null;
            status = String.format(Locale.US, "APPLIED from autonomous: X %.2f m | Y %.2f m | H %.1f deg",
                    applied.x, applied.y, Math.toDegrees(applied.heading));
            return applied.toPose2D();
        }
        /** Discards a pose that has not been applied yet (corner rehome, START before the Pinpoint settled). */
        public void cancel(String reason) {
            if (pose == null) return;
            pose = null;
            status = reason;
        }
        public boolean pending() { return pose != null; }
        public String status() { return status; }
    }
}
