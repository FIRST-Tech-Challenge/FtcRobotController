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
 * repo's PlannerRobot, which runs competition TeleOp's own shared code (CompetitionShotPipeline): ShotCoordinator
 * aim (RT) with hood / RPM from ShootingConfiguration.PROFILE, FiringCoordinator readiness and continuous feed
 * (RT + LT, GP1 A), IntakeQueueController collection, TransferSubsystem loading, GateSubsystem, the bang-bang
 * flywheel; turret homing; Limelight tag relocalization (T.TAG_RELOCALIZATION: INIT start-pose refinement +
 * aim-pose correction, like BioBuzzRobot; a Relocalize step repeats it mid-route); and the optional
 * depollinator, linear slide (shot blocker), status lights and pollen camera (an unconfigured device is
 * reported in telemetry and its steps are skipped).
 *
 * - Coordinates: the repo's field frame (PedroDrive, Zenith routes, TeleOpFieldReference): inches,
 *   (0, 0) = field center, +x toward the blue wall, +y away from the audience. Headings in degrees, CCW positive.
 * - Planned alliance RED.
 * - Robot placed as a 23 x 12 in box, center (-12.41, -65.39), heading 179.8 deg.
 * - Start pose (tracked point = box center + pod offsets X 0 fwd, Y 0 reversed): (-12.415, -65.386).
 * - Path limits: speed is a fraction of PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC; max velocity,
 *   acceleration and deceleration are Foresight constraints for that one path (0 = no limit).
 *
 * HOW IT RUNS: ONE while (opModeIsActive()) loop: loop.begin() (bulk cache), drive.readSensors() (the single
 * Pinpoint read), robot.readSensors(), the current step, loop-code blocks, drive.update() (Pedro, right after
 * the step so the pose it uses is as fresh as possible), then robot.apply() (same-loop mechanism writes).
 * A drive step ends SETTLED (drive.settled(): Pedro's end hold converged) unless set otherwise; with
 * T.PATHS_FROM_ROBOT_POSE a path starts where the robot is. T.MECHANISMS_ENABLED = false is a drive-only
 * A/B test. Steps wait for a fresh, referenced Pinpoint pose. Like the
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
        public static double SETTLE_TIMEOUT_S = 0.5;
        public static boolean PATHS_FROM_ROBOT_POSE = true;
        public static boolean MECHANISMS_ENABLED = true;
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
    /** First point of a path: where the robot is now (T.PATHS_FROM_ROBOT_POSE), else the drawn point. */
    private Pose start(Pose drawn) { return T.PATHS_FROM_ROBOT_POSE ? follower.pose().withHeading(0) : drawn; }
    /** Start heading of a linear-heading path: the robot's heading now (T.PATHS_FROM_ROBOT_POSE), else the drawn one. */
    private double startH(double drawn) { return T.PATHS_FROM_ROBOT_POSE ? follower.pose().heading() : drawn; }
    /** 0..1 progress through the path being followed, spread evenly over its segments (1 once it ended). */
    private double progress() {
        if (!follower.following()) return 1.0;
        return clamp((follower.pathIndex() + follower.curveCompletion()) / Math.max(1, pathSegs), 0, 1);
    }
    /** Per-path Foresight limits. speed = fraction of max forward velocity; the rest are 0 = no limit. */
    private Path limits(Path path, double speed, double maxVelInPerS, double accelInPerS2, double decelInPerS2) {
        List<Modifier> m = new ArrayList<>();
        // Never plan faster than the wheel cap allows (no cap at T.MAX_WHEEL_POWER = 1). A cap below 1 also caps braking,
        // so Foresight's full-power stop plan overshoots and the robot drives back (seen at a 0.35 cap, 2026-11-19).
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
        // V1 tuner run, 2026-11-19) at T.MAX_WHEEL_POWER (1 = full power, the power the tuner measured braking at).
        drive = new PedroDrive(hardwareMap, true, T.MAX_WHEEL_POWER);
        follower = drive.follower();
        cfg = drive.config();
        try {
            robot = new PlannerRobot(hardwareMap, drive, alliance(), T.TAG_RELOCALIZATION, false);   // starts turret homing
            robot.intakePowerScale = T.INTAKE_POWER;
            robot.feedSeconds = T.FEED_SECONDS;
            robot.readyTimeoutSeconds = T.READY_TIMEOUT_S;
            drive.setStartPose(P(-12.415, -65.386, 179.8));   // box center (-12.41, -65.39) + pod offsets
            boolean lastMirrored = mirrored;
            while (opModeInInit()) {
                loop.begin();
                drive.readSensors();   // applies the start pose once Pinpoint reports READY
                double t = now();   // after the Pinpoint read: pose freshness needs t >= sample time
                if (T.MECHANISMS_ENABLED) {
                    robot.readSensors(t);
                    robot.initLoop(t);   // turret homing + tag relocalization; nothing spins or feeds before START
                }
                telemetry.addData("Alliance", alliance());
                telemetry.addLine("BioBuzz auto ready - keep the robot still during INIT (Pinpoint, turret homing, tags)");
                telemetry.addData("Pose ready", drive.ready(now()));
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                telemetry.update();
            }
            if (isStopRequested()) return;
            robot.onStart();   // ends a start-pose refinement still collecting frames

            final int STEPS = 7;
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
                if (T.MECHANISMS_ENABLED) robot.readSensors(t);
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

                drive.update();   // Pedro uses this loop's Pinpoint sample
                if (T.MECHANISMS_ENABLED) robot.apply(t);   // TeleOp's shot pipeline, intake, transfer, lights
                telemetry.addData("step", "%d / %d%s | %.1f s", Math.min(stage, STEPS), STEPS, poseSeen ? "" : " (waiting for Pinpoint pose)", t - started);
                telemetry.addData("Alliance", alliance());
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                loop.addTelemetry(telemetry);   // loop period / Hz and work time
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
                if (!T.MECHANISMS_ENABLED) return true;   // A/B drive test: robot-function steps off
                if (first) {
                    robot.setTrims(0, 0, 0);   // auto-aim trims: yaw deg (left +), hood deg, RPM
                    robot.startVolley(P(T.GOAL1_X, T.GOAL1_Y, 0), T.FEED_SECONDS, T.READY_TIMEOUT_S, PlannerVolley.TimeoutAction.FIRE, PlannerRobot.After.KEEP);   // TeleOp RT aim, RT + LT fire
                }
                if (T.BRAKE_WHILE_SHOOTING && !follower.idle() && (drive.settled() || stepTimer.seconds() > T.SETTLE_TIMEOUT_S)) drive.stop();
                return !first && robot.volleyDone();
            }

            // step 2 - DRIVE to (-47.7, -24.4) @ 179.8 deg
            case 1: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(start(P(-12.41, -65.39, 0)), P(-47.67, -24.42, 0)).linear(startH(H(179.8)), H(179.8)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (drive.settled() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 3 - ACTION DEPOLLINATOR: deploy · wait settle
            case 2: {
                if (!T.MECHANISMS_ENABLED) return true;   // A/B drive test: robot-function steps off
                if (first) robot.deployDepollinator();
                return !first && (robot.depollinatorSettled() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
            }

            // step 4 - DRIVE to (-52, -24.4) @ 179.8 deg
            case 3: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(start(P(-47.67, -24.42, 0)), P(-52.04, -24.42, 0)).linear(startH(H(179.8)), H(179.8)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (drive.settled() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 5 - ACTION INTAKE: in
            case 4: {
                if (!T.MECHANISMS_ENABLED) return true;   // A/B drive test: robot-function steps off
                if (first) robot.setIntake(PlannerRobot.IntakeMode.IN, 0, false);   // off after (s, 0 = stays on), off once full
                return true;
            }

            // step 6 - DRIVE to (-11.9, -58.9) @ 90 deg
            case 5: {
                if (first) {
                    pathSegs = 1;
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(start(P(-52.04, -24.42, 0)), P(-11.93, -58.93, 0)).linear(startH(H(179.8)), H(90)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (drive.settled() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 7 - ACTION VOLLEY (AIM + SPIN + SHOOT): feed 2.5s at aim point 1 (red audience)
            case 6: {
                if (!T.MECHANISMS_ENABLED) return true;   // A/B drive test: robot-function steps off
                if (first) {
                    robot.setTrims(0, 0, 0);   // auto-aim trims: yaw deg (left +), hood deg, RPM
                    robot.startVolley(P(T.GOAL1_X, T.GOAL1_Y, 0), T.FEED_SECONDS, T.READY_TIMEOUT_S, PlannerVolley.TimeoutAction.FIRE, PlannerRobot.After.KEEP);   // TeleOp RT aim, RT + LT fire
                }
                if (T.BRAKE_WHILE_SHOOTING && !follower.idle() && (drive.settled() || stepTimer.seconds() > T.SETTLE_TIMEOUT_S)) drive.stop();
                return !first && robot.volleyDone();
            }

            default:
                return true;
        }
    }
}
