package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Generated with the BioBuzz Path Planner (FTC 2026-27 "BIOBUZZ" presented by RTX) - Pedro Pathing 3 target.
 *
 * Built for com.pedropathing:core 3.0.0 (Paths API). The drivetrain, Pinpoint and Pedro follower come
 * from the repo's PedroDrive, so its PedroConfiguration tuning applies. Every mechanism is a repo
 * subsystem, driven like the competition and fallback autos: IntakeSubsystem (TeleOp's direction),
 * TransferSubsystem (full sensor, stage 1 stall guard), GateSubsystem with the shared FiringCoordinator
 * feed, and TurretSubsystem (homing, rear dead zone, encoder-verified yaw, shooter PIDF), aimed with the
 * competition pivot geometry and aim lead; hood / RPM come from ShootingConfiguration.PROFILE.
 *
 * - Coordinates: the repo's field frame (PedroDrive, Zenith routes, TeleOpFieldReference): inches,
 *   (0, 0) = field center, +x toward the blue wall, +y away from the audience. Headings in degrees, CCW positive.
 * - Planned alliance RED.
 * - Robot placed as a 23 x 12 in box, center (-11.85, -64.42), heading -179.2 deg.
 * - Start pose (tracked point = box center + pod offsets X 0 fwd, Y 0 reversed): (-11.846, -64.424).
 * - Path limits: speed is a fraction of PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC; max velocity,
 *   acceleration and deceleration are Foresight constraints for that one path (0 = no limit).
 *
 * HOW IT RUNS: ONE while (opModeIsActive()) loop: loop.begin() (bulk cache), drive.readSensors() (the single
 * Pinpoint read), robot.readSensors(), the current step, loop-code blocks, robot.apply() (same-loop
 * mechanism writes), then drive.update(). Steps wait for a fresh, referenced Pinpoint pose. Like the
 * repo's autos it clears AutoPoseHandoff in INIT and saves the pose every loop for TeleOp, stops at
 * PedroConfiguration.MAX_AUTONOMOUS_SECONDS, and stops if the pose stays stale for
 * FallbackAutoController.POSE_LOSS_SECONDS. Keep the robot still during INIT (Pinpoint IMU calibration
 * and turret homing). In your own code use: follower, drive, robot, telemetry, stepTimer, P(x, y, deg),
 * H(deg), progress().
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
        // ---- Gate, kicker & transfer (shot timing) ----
        public static double FEED_SECONDS = FallbackAutoController.FEED_SECONDS;   // repo value
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
    private Robot robot;
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
        m.add(cfg.setPathSpeed(speed));
        if (maxVelInPerS > 0) m.add(cfg.maxVelocityConstraint.at(maxVelInPerS));
        if (accelInPerS2 > 0) m.add(cfg.maxAccelerationConstraint.at(accelInPerS2));
        if (decelInPerS2 > 0) m.add(cfg.maxDecelerationConstraint.at(decelInPerS2));
        return path.with(m);
    }
    private static double now() { return System.nanoTime() * 1e-9; }
    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    static double wrap(double a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

    // =====================================================================================
    //  ROBOT - the repo's subsystems, driven the way the competition and fallback autos drive them:
    //  IntakeSubsystem (TeleOp's direction), TransferSubsystem (full sensor, stage 1 stall guard),
    //  GateSubsystem with the shared FiringCoordinator feed (CompetitionFeedController timing), and
    //  TurretSubsystem for yaw, hood and flywheel. Call readSensors() once per loop before the step
    //  and apply() once per loop after it (the OpMode does both).
    // =====================================================================================
    static class Robot {
        static final double METERS_PER_INCH = FieldGeometry.METERS_PER_INCH;

        final IntakeSubsystem intake;
        final TransferSubsystem transfer;
        final GateSubsystem gate;
        final TurretSubsystem turret;
        final CompetitionFeedController feedControl = new CompetitionFeedController(); // continuous feed
        final FiringCoordinator firing = CompetitionFeedController.firing();

        double intakePower = 0;              // + in, - out, in IntakeSubsystem's (TeleOp's) direction
        boolean gateHeldOpen = false;        // Gate step: hold the gate open outside a shot
        double feedUntil = Double.NEGATIVE_INFINITY; // a shot's feed request runs until this time (s)

        boolean spinning = false;
        boolean rpmAuto = true;
        double rpmManual = 0;                // flywheel output RPM
        double rpmTarget = 0;

        Pose aimTarget = null;               // point the turret tracks / range is measured to (field in)
        boolean tracking = false;
        double fixedHeadingDeg = Double.NaN; // fixed turret heading: front 0, right 90, back 180, left 270
        boolean poseFresh = false;
        double dist = Double.NaN;            // turret pivot to aim point, inches (NaN until an aim point is set)

        boolean hoodAuto = true;
        double hoodManualDeg = TurretSubsystem.PITCH_HOME_DEG; // hood travel down from home, 0..30 deg
        double hoodTargetDeg = TurretSubsystem.PITCH_HOME_DEG;

        String shotNote = "";
        int homeRetries = 0;                 // turret homes restarted after a FAULT

        Robot(HardwareMap hw) {
            intake = new IntakeSubsystem(hw);        // never resets intake_motor's encoder (turret encoder channel)
            transfer = new TransferSubsystem(hw);
            gate = new GateSubsystem(hw, CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS);
            // Starts non-blocking turret homing and puts the hood at home; no camera in this auto.
            turret = new TurretSubsystem(hw, false);
            turret.setEncoderCorrectionEnabled(true);
        }

        // ---- commands (call these from code steps / loop code too) ----
        /** Intake in (1), out (-1) or off (0) at T.INTAKE_POWER; forward collection stops once the transfer is full. */
        void setIntake(int mode) { intakePower = mode * T.INTAKE_POWER; }
        void spin(boolean on) { spinning = on; }
        void setRpmAuto() { rpmAuto = true; }
        /** Flywheel output RPM, the same units as TeleOp and the shot table. */
        void setRpmManual(double rpm) { rpmAuto = false; rpmManual = rpm; }
        void setHoodAuto() { hoodAuto = true; }
        /** Hood travel down from home in degrees (0 = home / highest launch, 30 = lowest). */
        void setHoodManual(double travelDeg) {
            hoodAuto = false;
            hoodManualDeg = clamp(travelDeg, TurretSubsystem.PITCH_MIN_DEG, TurretSubsystem.PITCH_MAX_DEG);
        }
        void aimAt(Pose target) { aimTarget = target; }
        void setTracking(boolean on) { tracking = on; if (on) fixedHeadingDeg = Double.NaN; }
        /** Fixed turret heading in the Driver Hub frame: front 0, right (home) 90, back 180, left 270 deg. */
        void setTurretHeading(double headingDeg) { tracking = false; fixedHeadingDeg = headingDeg; }
        /** Hold the gate open (true) or let the feed control it again (false). */
        void gateOpen(boolean open) { gateHeldOpen = open; }

        /**
         * Shot: the fallback auto's continuous feed (gate, then both transfer stages) for T.FEED_SECONDS. A
         * continuous feed cannot count launched balls (FiringCoordinator), so it feeds every loaded ball.
         */
        void startShoot() { feedUntil = now() + T.FEED_SECONDS; shotNote = ""; }
        boolean isShooting() { return now() < feedUntil; }
        boolean turretHomed() { return turret.isHomed(); }

        /** Encoder-verified turret on its target (homed, fresh pose while tracking). */
        boolean turretReady() {
            return turret.isHomed() && (!tracking || poseFresh)
                    && Math.abs(turret.getTargetYawDegrees() - turret.getEncoderAngleDegrees())
                    <= TurretSubsystem.SHOT_READY_YAW_TOLERANCE_DEG;
        }

        /** Same readiness as the fallback auto: flywheel at speed, hood settled and turret on target. */
        boolean ready() {
            return spinning && rpmTarget > 0 && turret.isShooterAtSpeed() && turret.isHoodTracking() && turretReady();
        }

        /** Mid-auto stop: mechanisms off, the turret holds its last target and keeps its home. */
        void stopAll() {
            intakePower = 0;
            spinning = false;
            tracking = false;
            fixedHeadingDeg = Double.NaN;
            gateHeldOpen = false;
            feedUntil = Double.NEGATIVE_INFINITY;
            double t = now();
            RobotShutdown.stopAll(() -> firing.stop(t), intake::stop, transfer::pause, gate::close, turret::stopShooter);
        }

        /** End of OpMode: every output stops even if one device throws (the fallback auto's set). */
        void shutdown() { RobotShutdown.stopAll(this::stopAll, transfer::stop, gate::close, turret::stop); }

        /**
         * Once per loop, before the step: transfer full sensor and stage 1 speed, turret encoder, homing and
         * flywheel velocity. A home that faults (turret bumped or still moving in its settle window) restarts
         * at once; otherwise the turret would stay unhomed and never aim for the rest of the auto.
         */
        void readSensors(double now) {
            transfer.readSensors(now);
            turret.update(now);
            if (turret.hasHomingFault()) { homeRetries++; turret.homeTurretAndZeroEncoder(now); }
        }

        /** INIT loop: turret homing travel only; nothing aims, spins or feeds before START. */
        void applyInit(double now) { turret.applyYawCommand(now); }

        /**
         * Once per loop, after the step. pose: this loop's Pinpoint sample (meters, field-center frame, CCW).
         * Aim uses the competition auto-aim rules: pivot offset from ShootingConfiguration.MOUNT, the
         * TurretControl aim lead, and TurretSubsystem's reachable-yaw rule for the rear dead zone.
         */
        void apply(double now, MotionSample pose) {
            poseFresh = pose != null && pose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS);
            ShooterMount mount = ShootingConfiguration.MOUNT;
            if (aimTarget != null && poseFresh) {
                double gx = aimTarget.x() * METERS_PER_INCH, gy = aimTarget.y() * METERS_PER_INCH;
                dist = TeleOpFieldReference.pivotRangeMeters(pose, gx, gy, mount) / METERS_PER_INCH;
                if (tracking && turret.isHomed()) {
                    double yawDeg = Math.toDegrees(TeleOpFieldReference.yawRadians(
                            pose.led(TurretControl.AIM_LEAD_SECONDS), gx, gy, mount));
                    turret.setTargetYawDegrees(turret.reachableYawTarget(yawDeg));
                }
            }
            if (!tracking && Double.isFinite(fixedHeadingDeg) && turret.isHomed()) {
                turret.setTargetYawDegrees(turret.reachableYawTarget(
                        TurretControl.robotHeadingToYawDegrees(fixedHeadingDeg)));
            }
            turret.applyYawCommand(now); // Same-loop servo write; also drives homing travel.

            // ---- hood + flywheel: the measured shot table (range from the turret pivot). Before any aim
            //      point, the fallback auto's preload settings (TeleOp's starting manual hood / RPM). ----
            ShotProfile.Row row = Double.isFinite(dist) ? ShootingConfiguration.PROFILE.sample(dist * METERS_PER_INCH) : null;
            hoodTargetDeg = !hoodAuto ? hoodManualDeg : row != null ? row.hoodDeg : FallbackAutonomous.PRELOAD_HOOD_DEG;
            turret.setPitchAngleDegrees(hoodTargetDeg);
            rpmTarget = !rpmAuto ? ShooterSpeed.clampFlywheelRpm(rpmManual)
                    : row != null ? row.rpm : FallbackAutonomous.PRELOAD_FLYWHEEL_RPM;
            if (spinning && rpmTarget > 0) turret.setShooterRpm(rpmTarget);
            else turret.stopShooter();

            // ---- feed: the fallback auto's manual-feed path (TeleOp GP1 A): gate, then both stages ----
            boolean feedRequested = now < feedUntil;
            feedControl.updateSelection(now, false, firing);
            firing.update(now, false, feedRequested, false, false, false, false, false, "Planner autonomous feed");
            boolean feedStage1 = feedControl.feedStage1(now, firing);
            gate.setOpen(firing.gateOpen() || gateHeldOpen);
            boolean shootingBusy = feedRequested || firing.gateOpen()
                    || firing.state() == FiringCoordinator.State.RECOVERY;
            // Stage 1 loads (full sensor + run-on, stall guard) while intaking or spinning up for a shot.
            boolean loading = intakePower > 0 || spinning;
            transfer.updateCached(now, feedControl.feedTransfers(firing, gate.isOpenForFeeding()),
                    shootingBusy || !loading, feedStage1, false);
            // Forward collection stops on a confirmed full queue (competition auto rule); reverse still allowed.
            intake.setIntakePower(transfer.isFull() && intakePower > 0
                    ? IntakeQueueController.FULL_QUEUE_INTAKE_POWER : intakePower);
            transfer.setIntakeRunning(intake.getCommandedPower() > 0); // Stage 1 stall guard only while intaking.
        }

        void addTelemetry(Telemetry t) {
            turret.addDriverTelemetry(t);
            transfer.addDriverTelemetry(t);
            if (homeRetries > 0) t.addData("Turret home retries", homeRetries);
            t.addData("Turret target heading (deg CW)", "%.1f | %s", turret.getTargetRobotHeadingDegrees(),
                    !turret.isHomed() ? "NOT HOMED" : turretReady() ? "ON TARGET" : tracking && !poseFresh ? "POSE STALE" : "MOVING");
            t.addData("Aim range (in)", Double.isFinite(dist)
                    ? String.format(java.util.Locale.US, "%.1f", dist) : "no aim point (preload settings)");
            t.addData("Hood travel (deg) / flywheel target (RPM)", "%.1f / %.0f", hoodTargetDeg, rpmTarget);
            t.addData("Shot", "ready %s | feed %s%s", ready(), firing.state(), shotNote.isEmpty() ? "" : " | " + shotNote);
        }
    }


    // =====================================================================================
    //  OPMODE
    // =====================================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        AutoPoseHandoff.clear();   // like every repo auto: an older run's pose must never reach the next TeleOp
        RobotLoop loop = new RobotLoop(hardwareMap);   // bulk-cached hub reads, cleared once per loop
        drive = new PedroDrive(hardwareMap);   // drive motors + Pinpoint + Pedro 3 follower
        follower = drive.follower();
        cfg = drive.config();
        try {
            robot = new Robot(hardwareMap);    // starts turret homing
            drive.setStartPose(P(-11.846, -64.424, -179.2));   // box center (-11.85, -64.42) + pod offsets
            boolean lastMirrored = mirrored;
            while (opModeInInit()) {
                loop.begin();
                drive.readSensors();   // applies the start pose once Pinpoint reports READY
                double t = now();   // after the Pinpoint read: pose freshness needs t >= sample time
                robot.readSensors(t);
                robot.applyInit(t);   // turret homing only; nothing aims, spins or feeds before START
                telemetry.addData("Alliance", alliance());
                telemetry.addLine("BioBuzz auto ready - keep the robot still during INIT (Pinpoint + turret homing)");
                telemetry.addData("Pose ready", drive.ready(now()));
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                telemetry.update();
            }
            if (isStopRequested()) return;

            final int STEPS = 6;
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

                robot.apply(t, drive.sample());
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

            // step 2 - ACTION STOP ALL: intake, flywheel, feed off
            case 1: {
                robot.stopAll();
                return true;
            }

            // step 3 - DRIVE to (-58.5, -63.9) @ -179.2 deg
            case 2: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-11.85, -64.42, 0), P(-58.53, -63.89, 0)).constant(H(-179.2)),
                            PedroConfiguration.DEFAULT_PATH_SPEED_FRACTION, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 4 - ACTION INTAKE: in · 2s
            case 3: {
                if (first) robot.setIntake(1);
                boolean done = stepTimer.seconds() >= 2;
                return done;
            }

            // step 5 - DRIVE to (-13.5, -50.3) @ -179.2 deg
            case 4: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(-58.53, -63.89, 0), P(-13.54, -50.29, 0)).constant(H(-179.2)),
                            PedroConfiguration.DEFAULT_PATH_SPEED_FRACTION, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 6 - ACTION VOLLEY (AIM + SPIN + SHOOT): feed 2.5s at aim point 1 (red audience)
            case 5: {
                if (first) {
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
