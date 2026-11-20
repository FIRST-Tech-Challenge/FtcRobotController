package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Generated with the BioBuzz Path Planner (FTC 2026-27 "BIOBUZZ" presented by RTX) - Pedro Pathing 3 target.
 *
 * Built for com.pedropathing:core 3.0.0 (Paths API). The drivetrain, Pinpoint and Pedro follower come
 * from the repo's PedroDrive, so its PedroConfiguration tuning applies. Every mechanism comes from the
 * repo's PlannerRobot, driven like the competition and fallback autos: intake, transfer (full sensor +
 * stage 1 run-on), gate + shared FiringCoordinator feed, turret (homing, rear dead zone, encoder-
 * verified yaw, shooter PIDF; hood / RPM from ShootingConfiguration.PROFILE), Limelight tag relocalization
 * (T.TAG_RELOCALIZATION: INIT start-pose refinement + aim-pose correction, like BioBuzzRobot), and the
 * optional depollinator, linear slide (shot blocker), status lights and pollen camera (an unconfigured device is
 * reported in telemetry and its steps are skipped).
 *
 * - Coordinates: the repo's field frame (PedroDrive, Zenith routes, TeleOpFieldReference): inches,
 *   (0, 0) = field center, +x toward the blue wall, +y away from the audience. Headings in degrees, CCW positive.
 * - Planned alliance RED.
 * - Robot placed as a 23 x 12 in box, center (-12.42, -65.39), heading 179.8 deg.
 * - Start pose (tracked point = box center + pod offsets X 0 fwd, Y 0 reversed): (-12.415, -65.386).
 * - Path limits: speed is a fraction of PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC; max velocity,
 *   acceleration and deceleration are Foresight constraints for that one path (0 = no limit).
 *
 * HOW IT RUNS: ONE while (opModeIsActive()) loop: loop.begin() (bulk cache), drive.readSensors() (the single
 * Pinpoint read), robot.readSensors(), the current step, loop-code blocks, robot.apply() (same-loop
 * mechanism writes), then drive.update(). Steps wait for a fresh, referenced Pinpoint pose. Like the
 * repo's autos it clears AutoPoseHandoff in INIT and saves the pose every loop for TeleOp, stops at
 * PedroConfiguration.MAX_AUTONOMOUS_SECONDS, and stops if the pose stays stale for
 * FallbackAutoController.POSE_LOSS_SECONDS. Keep the robot still during INIT (Pinpoint IMU calibration,
 * turret homing and tag relocalization) with the slide fully down. In your own code use: follower, drive,
 * robot (PlannerRobot), telemetry, stepTimer, P(x, y, deg), H(deg), progress().
 */

@Autonomous(name = "testdrive", group = "Auto")
public class testdrive extends LinearOpMode {

    // =====================================================================================
    //  TUNING - change here, or live from FTC Dashboard if @Config is enabled
    // =====================================================================================
    public static class T {
        // ---- Intake ----
        public static double INTAKE_POWER = 1;
        // ---- Shot readiness ----
        public static double READY_TIMEOUT_S = FallbackAutoController.SPIN_UP_TIMEOUT_SECONDS;   // repo value
        // ---- Gate & transfer (shot timing) ----
        public static double FEED_SECONDS = FallbackAutoController.FEED_SECONDS;   // repo value
        // ---- Limelight ----
        public static boolean TAG_RELOCALIZATION = BioBuzzRobot.TAG_RELOCALIZATION_ENABLED;   // repo value
        // ---- Aim points (field-center inches, planned side) ----
        public static double GOAL1_X = TurretSubsystem.RED_HIVE_PIVOT_X_IN;   // repo value
        public static double GOAL1_Y = -TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN;   // repo value
        public static double GOAL2_X = TurretSubsystem.RED_HIVE_PIVOT_X_IN;   // repo value
        public static double GOAL2_Y = TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN;   // repo value
        public static double GOAL3_X = TurretSubsystem.BLUE_HIVE_PIVOT_X_IN;   // repo value
        public static double GOAL3_Y = -TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN;   // repo value
        public static double GOAL4_X = TurretSubsystem.BLUE_HIVE_PIVOT_X_IN;   // repo value
        public static double GOAL4_Y = TurretSubsystem.HIVE_RAISED_MOUTH_CENTER_Y_OFFSET_IN;   // repo value
        // ---- Driving & generator ----
        public static double STEP_TIMEOUT_S = 0;
        public static boolean BRAKE_WHILE_SHOOTING = true;
        public static double MAX_WHEEL_POWER = PedroConfiguration.MAX_WHEEL_POWER;   // repo value
        // ---- Pedro 3 path limits (driver defaults; a step can override them) ----
        public static double MAX_ACCEL_IN_PER_S2 = 0;   // 0 = no limit
        public static double MAX_DECEL_IN_PER_S2 = 0;   // 0 = no limit
        public static double TURN_TOLERANCE_DEG = Math.toDegrees(PedroConfiguration.END_HEADING_TOLERANCE_RAD);   // repo value
    }

    private static final boolean MIRROR = false;   // INIT alliance select (B = other alliance)
    private static final TurretSubsystem.Alliance PLANNED_ALLIANCE = TurretSubsystem.Alliance.RED;

    private PedroDrive drive;
    private Follower follower;
    private ForesightConfig cfg;
    private PlannerRobot robot;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private boolean mirrored = false;
    private int phase = 0;
    private int pathSegs = 1;
    private Pose turnGoal = Pose.zero();
    private final boolean[] cbFired = new boolean[1];

    // ---- helpers (alliance mirroring is applied here, so planner coordinates stay as drawn) ----
    /** Planner pose (field-center inches, deg) -> Pedro pose; the other alliance is the point mirror. */
    private Pose P(double x, double y, double deg) {
        double h = Math.toRadians(deg);
        if (mirrored) { x = -x; y = -y; h += Math.PI; }   // same as BioBuzzRobot.mirror()
        return new Pose(x, y, wrap(h));
    }
    private double H(double deg) { return P(0, 0, deg).heading(); }
    private TurretSubsystem.Alliance alliance() {
        if (!mirrored) return PLANNED_ALLIANCE;
        return PLANNED_ALLIANCE == TurretSubsystem.Alliance.RED ? TurretSubsystem.Alliance.BLUE : TurretSubsystem.Alliance.RED;
    }
    private double distTo(Pose p) { Pose c = follower.pose(); return Math.hypot(c.x() - p.x(), c.y() - p.y()); }
    /** 0..1 progress through the path being followed, spread evenly over its segments (1 once it ended). */
    private double progress() {
        if (!follower.following()) return 1.0;
        return clamp((follower.pathIndex() + follower.curveCompletion()) / Math.max(1, pathSegs), 0, 1);
    }
    /** Per-path Foresight limits. speed = fraction of max forward velocity; the rest are 0 = no limit. */
    private Path limits(Path path, double speed, double maxVelInPerS, double accelInPerS2, double decelInPerS2) {
        List<Modifier> m = new ArrayList<>();
        // Never plan faster than the capped wheels can drive: at speed 1.0 with a 0.35 wheel cap, Foresight plans
        // a ~70 in/s profile the robot cannot follow, and it overshot sideways without settling (2026-11-19).
        m.add(cfg.setPathSpeed(Math.min(speed, T.MAX_WHEEL_POWER)));
        if (maxVelInPerS > 0) m.add(cfg.maxVelocityConstraint.at(maxVelInPerS));
        if (accelInPerS2 > 0) m.add(cfg.maxAccelerationConstraint.at(accelInPerS2));
        if (decelInPerS2 > 0) m.add(cfg.maxDecelerationConstraint.at(decelInPerS2));
        return path.with(m);
    }
    private static double now() { return System.nanoTime() * 1e-9; }
    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

    // =====================================================================================
    //  OPMODE
    // =====================================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        AutoPoseHandoff.clear();   // like every repo auto: an older run's pose must never reach the next TeleOp
        RobotLoop loop = new RobotLoop(hardwareMap);   // bulk-cached hub reads, cleared once per loop
        // Drive motors + Pinpoint + Pedro 3 follower with the PedroConfiguration Foresight tuning (the
        // V1 tuner run, 2026-11-19), capped at T.MAX_WHEEL_POWER (0.35 until V1's paths are verified).
        drive = new PedroDrive(hardwareMap, true, T.MAX_WHEEL_POWER);
        follower = drive.follower();
        cfg = drive.config();
        try {
            robot = new PlannerRobot(hardwareMap, drive, alliance(), T.TAG_RELOCALIZATION, false);   // starts turret homing
            robot.intakePowerScale = T.INTAKE_POWER;
            robot.feedSeconds = T.FEED_SECONDS;
            drive.setStartPose(P(-12.415, -65.386, 179.8));   // box center (-12.42, -65.39) + pod offsets
            boolean lastMirrored = mirrored;
            while (opModeInInit()) {
                loop.begin();
                drive.readSensors();   // applies the start pose once Pinpoint reports READY
                double t = now();   // after the Pinpoint read: pose freshness needs t >= sample time
                robot.readSensors(t);
                robot.initLoop(t);   // turret homing + tag relocalization; nothing spins or feeds before START
                telemetry.addData("Alliance", alliance());
                telemetry.addLine("BioBuzz auto ready - keep the robot still during INIT (Pinpoint, turret homing, tags)");
                telemetry.addData("Pose ready", drive.ready(now()));
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                telemetry.update();
            }
            if (isStopRequested()) return;
            robot.onStart();   // ends a start-pose refinement still collecting frames

            final int STEPS = 8;
            final double started = now();
            int stage = 0;               // steps started so far; running step index = stage - 1
            boolean running = false;
            boolean first = false;
            boolean poseSeen = false;
            double poseLostSince = Double.NaN;
            String endReason = "Auto complete";

            while (opModeIsActive() && (stage < STEPS || running)) {
                loop.begin();
                drive.readSensors();
                double t = now();   // after the Pinpoint read: pose freshness needs t >= sample time
                robot.readSensors(t);
                if (t - started >= PedroConfiguration.MAX_AUTONOMOUS_SECONDS) {
                    endReason = "Stopped at the autonomous deadline";
                    break;
                }
                if (drive.ready(t)) {
                    poseSeen = true;
                    poseLostSince = Double.NaN;
                    AutoPoseHandoff.save(alliance(), drive.sample());   // TeleOp can start from the last good pose
                } else if (poseSeen) {
                    if (Double.isNaN(poseLostSince)) poseLostSince = t;
                    if (t - poseLostSince >= FallbackAutoController.POSE_LOSS_SECONDS) {
                        AutoPoseHandoff.clear();   // a pose from before a localization fault is not trustworthy
                        endReason = "FAULT: Pinpoint pose unavailable / stale; stopped";
                        break;
                    }
                }

                if (poseSeen && !running && stage < STEPS) {
                    stage++;
                    running = true;
                    first = true;
                    phase = 0;
                    stepTimer.reset();
                }
                if (running) {
                    boolean done = runStep(stage - 1, first);
                    first = false;
                    if (done) running = false;
                }

                robot.apply(t);
                drive.update();
                telemetry.addData("step", "%d / %d%s | %.1f s", Math.min(stage, STEPS), STEPS, poseSeen ? "" : " (waiting for Pinpoint pose)", t - started);
                telemetry.addData("Alliance", alliance());
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                loop.addRateTelemetry(telemetry);
                telemetry.update();
            }
            RobotShutdown.stopAll(drive::stop, robot::stopAll);
            telemetry.addLine(endReason);
            telemetry.update();
        } finally {
            RobotShutdown.stopAll(drive::stop, robot == null ? null : robot::shutdown);
        }
    }

    /** One planner step. first = true on the first call. Returns true when the step is finished. */
    private boolean runStep(int i, boolean first) {
        switch (i) {

            // step 1 - ACTION VOLLEY (AIM + SPIN + SHOOT): feed 2.5s at aim point 1 (red audience)
            case 0: {
                if (first) {
                    if (T.BRAKE_WHILE_SHOOTING) drive.stop();   // zero drive power + BRAKE while shooting (no Pedro hold)
                    robot.aimAt(P(T.GOAL1_X, T.GOAL1_Y, 0));
                    robot.setTracking(true);
                    robot.spin(true);
                    robot.setRpmAuto();
                    robot.setHoodAuto();
                }
                if (phase == 0) {
                    boolean timedOut = stepTimer.seconds() > T.READY_TIMEOUT_S;
                    boolean ready = !first && robot.ready();   // first call: new targets apply after this step
                    if (ready || (timedOut && robot.turretHomed())) {
                        robot.startShoot();   // continuous feed of every loaded ball for T.FEED_SECONDS
                        if (!ready) robot.shotNote = "fed before READY (timeout)";
                        phase = 1;
                    } else if (timedOut) {
                        robot.shotNote = "SKIPPED: turret not homed";
                        return true;
                    }
                    return false;
                }
                if (!robot.isShooting()) {
                    return true;
                }
                return false;
            }

            // step 2 - ACTION INTAKE: in · 5s
            case 1: {
                if (first) robot.setIntake(1);
                boolean done = stepTimer.seconds() >= 5;
                return done;
            }

            // step 3 - DRIVE to (-59.1, -65.5) @ 179.8 deg
            case 2: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-12.42, -65.39, 0), P(-59.08, -65.52, 0)).constant(H(179.8)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 4 - DRIVE to (-47.7, -42) @ 100 deg
            case 3: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-59.08, -65.52, 0), P(-47.74, -42.01, 0)).constant(H(100)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 5 - DRIVE to (-58.4, -25.2) @ 175 deg
            case 4: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-47.74, -42.01, 0), P(-58.38, -25.15, 0)).constant(H(175)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 6 - DRIVE to (-39.8, 39.6) @ 74 deg
            case 5: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-58.38, -25.15, 0), P(-39.75, 39.59, 0)).constant(H(74)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 7 - DRIVE to (-14.2, 42.3) @ 5.9 deg
            case 6: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-39.75, 39.59, 0), P(-14.21, 42.25, 0)).constant(H(5.9)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 8 - ACTION VOLLEY (AIM + SPIN + SHOOT): feed 2.5s at aim point 1 (red audience)
            case 7: {
                if (first) {
                    if (T.BRAKE_WHILE_SHOOTING) drive.stop();   // zero drive power + BRAKE while shooting (no Pedro hold)
                    robot.aimAt(P(T.GOAL1_X, T.GOAL1_Y, 0));
                    robot.setTracking(true);
                    robot.spin(true);
                    robot.setRpmAuto();
                    robot.setHoodAuto();
                }
                if (phase == 0) {
                    boolean timedOut = stepTimer.seconds() > T.READY_TIMEOUT_S;
                    boolean ready = !first && robot.ready();   // first call: new targets apply after this step
                    if (ready || (timedOut && robot.turretHomed())) {
                        robot.startShoot();   // continuous feed of every loaded ball for T.FEED_SECONDS
                        if (!ready) robot.shotNote = "fed before READY (timeout)";
                        phase = 1;
                    } else if (timedOut) {
                        robot.shotNote = "SKIPPED: turret not homed";
                        return true;
                    }
                    return false;
                }
                if (!robot.isShooting()) {
                    return true;
                }
                return false;
            }

            default:
                return true;
        }
    }
}
