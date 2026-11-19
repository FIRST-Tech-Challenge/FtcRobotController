package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Generated with the BioBuzz Path Planner (FTC 2026-27 "BIOBUZZ" presented by RTX) - Pedro Pathing 3 target.
 *
 * Built for com.pedropathing:core 3.0.0 (Paths API). The drivetrain, Pinpoint and Pedro follower come
 * from the repo's PedroDrive, so its PedroConfiguration tuning applies. The turret yaw, hood and flywheel
 * are the repo's TurretSubsystem (homing, rear dead zone, encoder-verified yaw, shooter PIDF), aimed with
 * the same pivot geometry and aim lead as competition auto aim, and the hood / RPM come from the
 * team-measured ShootingConfiguration.PROFILE. Intake, gate, kicker and transfer are the Robot class below.
 *
 * - Coordinates: Pedro field inches, bottom-left origin, +x right, +y up. Headings in degrees, CCW positive.
 * - Robot placed as a 23 x 12 in box, center (60.15, 7.58), heading -179.2 deg.
 * - Start pose (tracked point = box center + pod offsets X 0 fwd, Y 0 reversed): (60.154, 7.576).
 * - Path limits: speed is a fraction of PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC; max velocity,
 *   acceleration and deceleration are Foresight constraints for that one path (0 = no limit).
 *
 * HOW IT RUNS: ONE while (opModeIsActive()) loop: loop.begin() (bulk cache), drive.readSensors() (the single
 * Pinpoint read), robot.readSensors(), the current step, loop-code blocks, robot.apply() (same-loop
 * turret/hood/flywheel writes), then drive.update(). Steps wait for a fresh, referenced Pinpoint pose.
 * Keep the robot still during INIT: Pinpoint IMU calibration and turret homing both run then.
 * In your own code use: follower, drive, robot, telemetry, stepTimer, P(x, y, deg), H(deg), progress().
 */

@Autonomous(name = "testdrive", group = "Auto")
public class testdrive extends LinearOpMode {

    // =====================================================================================
    //  TUNING - change here, or live from FTC Dashboard if @Config is enabled
    // =====================================================================================
    public static class T {
        // ---- Hardware names (must match the robot config) ----
        public static String INTAKE_NAME = "intake_motor";
        public static String TRANSFER_NAME = "transfer_motor_1";
        public static String KICKER_NAME = "transfer_motor_2";
        public static String GATE_NAME = "gate_servo";
        // ---- Motor directions ----
        public static boolean INTAKE_REVERSE_FROM_TELEOP = true;
        // ---- Intake ----
        public static double INTAKE_POWER = 1;
        // ---- Shot readiness ----
        public static double READY_TIMEOUT_S = 3;
        // ---- Gate, kicker & transfer (shot timing) ----
        public static double GATE_CLOSED = 0.5;
        public static double GATE_OPEN = 0.7;
        public static double GATE_OPEN_DELAY_S = 0.25;
        public static double FEED_PER_BALL_S = 0.3;
        public static double GATE_CLOSE_DELAY_S = 0.05;
        public static double KICKER_POWER = 1;
        public static double TRANSFER_POWER = 1;
        // ---- Aim points (field inches, planned side) ----
        public static double GOAL1_X = 60;
        public static double GOAL1_Y = 52;
        public static double GOAL2_X = 60;
        public static double GOAL2_Y = 92;
        public static double GOAL3_X = 84;
        public static double GOAL3_Y = 52;
        public static double GOAL4_X = 84;
        public static double GOAL4_Y = 92;
        public static double FALLBACK_DIST_IN = 70;
        // ---- Driving & generator ----
        public static double STEP_TIMEOUT_S = 0;
        // ---- Pedro 3 path limits (driver defaults; a step can override them) ----
        public static double MAX_ACCEL_IN_PER_S2 = 0;   // 0 = no limit
        public static double MAX_DECEL_IN_PER_S2 = 0;   // 0 = no limit
        public static double TURN_TOLERANCE_DEG = 3.0;
    }

    private static final double MIRROR_X = Double.NaN;   // NaN = off
    private static final double MIRROR_Y = Double.NaN;
    private static final boolean FLIP_TURNS = false;   // one mirror axis swaps turn directions

    private PedroDrive drive;
    private Follower follower;
    private ForesightConfig cfg;
    private Robot robot;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private boolean mirrored = false;
    private int phase = 0;
    private Pose turnGoal = Pose.zero();
    private final boolean[] cbFired = new boolean[1];

    // ---- helpers (alliance mirroring is applied here, so planner coordinates stay as drawn) ----
    private Pose P(double x, double y, double deg) {
        double h = Math.toRadians(deg);
        if (mirrored) {
            if (!Double.isNaN(MIRROR_X)) { x = 2 * MIRROR_X - x; h = Math.PI - h; }
            if (!Double.isNaN(MIRROR_Y)) { y = 2 * MIRROR_Y - y; h = -h; }
        }
        return new Pose(x, y, wrap(h));
    }
    private double H(double deg) { return P(0, 0, deg).heading(); }
    private boolean turnLeft(boolean left) { return (mirrored && FLIP_TURNS) ? !left : left; }
    private double distTo(Pose p) { Pose c = follower.pose(); return Math.hypot(c.x() - p.x(), c.y() - p.y()); }
    /** 0..1 progress through the path that is currently being followed. */
    private double progress() { return clamp(follower.curveCompletion(), 0, 1); }
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
    //  ROBOT SUBSYSTEMS - intake, gate, kicker + transfer here; turret yaw, hood and flywheel
    //  through TurretSubsystem. Call readSensors() once per loop before the step and apply()
    //  once per loop after it (the OpMode does both).
    // =====================================================================================
    static class Robot {
        static final double METERS_PER_INCH = .0254;

        final DcMotorEx intake, transfer, kicker;
        final Servo gate;
        final TurretSubsystem turret;
        final ElapsedTime clock = new ElapsedTime();

        int intakeMode = 0;                  // 1 in, -1 out, 0 off

        boolean spinning = false;
        boolean rpmAuto = true;
        double rpmManual = 0;                // flywheel output RPM
        double rpmTarget = 0;

        Pose aimTarget = null;               // point the turret tracks / distance is measured to (field in)
        boolean tracking = false;
        double fixedHeadingDeg = Double.NaN; // fixed turret heading: front 0, right 90, back 180, left 270
        boolean poseFresh = false;
        double dist = T.FALLBACK_DIST_IN;    // turret pivot to aim point, inches

        boolean hoodAuto = true;
        double hoodManualDeg = TurretSubsystem.PITCH_HOME_DEG; // hood travel down from home, 0..30 deg
        double hoodTargetDeg = TurretSubsystem.PITCH_HOME_DEG;

        int shootState = 0;                  // 0 idle, 1 gate opening, 2 feeding, 3 closing
        int shootBalls = 0;
        double shootT = 0;
        String shotNote = "";

        Robot(HardwareMap hw) {
            intake = hw.get(DcMotorEx.class, T.INTAKE_NAME);
            transfer = hw.get(DcMotorEx.class, T.TRANSFER_NAME);
            kicker = hw.get(DcMotorEx.class, T.KICKER_NAME);
            gate = hw.get(Servo.class, T.GATE_NAME);

            // Directions come from the subsystems TeleOp uses. The intake can be flipped from TeleOp's
            // direction with T.INTAKE_REVERSE_FROM_TELEOP (needed on the robot, 2026-11-18).
            // intake_motor's encoder channel is the turret's through-bore encoder. Never reset it here.
            setup(intake, T.INTAKE_REVERSE_FROM_TELEOP
                    ? IntakeSubsystem.MOTOR_DIRECTION.inverted() : IntakeSubsystem.MOTOR_DIRECTION);
            setup(transfer, TransferSubsystem.MOTOR_DIRECTION);
            setup(kicker, TransferSubsystem.STAGE_2_MOTOR_DIRECTION);
            gate.setPosition(T.GATE_CLOSED);

            // Starts non-blocking turret homing and puts the hood at home; no camera in this auto.
            turret = new TurretSubsystem(hw, false);
            turret.setEncoderCorrectionEnabled(true);
        }

        private static void setup(DcMotorEx m, DcMotorSimple.Direction direction) {
            m.setDirection(direction);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ---- commands (call these from code steps / loop code too) ----
        /** Intake in (1) also runs transfer stage 1 to carry balls toward the turret; out (-1) / off (0) stop it. */
        void setIntake(int mode) {
            intakeMode = mode;
            intake.setPower(mode * T.INTAKE_POWER);
            if (shootState != 2) transfer.setPower(stage1IdlePower());
        }
        /** Stage 1 outside the feed window: runs only while intaking. */
        private double stage1IdlePower() { return intakeMode == 1 ? T.TRANSFER_POWER : 0; }
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
        void gateOpen(boolean open) { gate.setPosition(open ? T.GATE_OPEN : T.GATE_CLOSED); }

        void startShoot(int balls) {
            shootBalls = Math.max(1, balls);
            shootState = 1;
            shootT = clock.seconds();
            shotNote = "";
        }
        boolean isShooting() { return shootState != 0; }
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
            shootState = 0;
            setIntake(0);                    // also stops transfer stage 1
            spinning = false;
            tracking = false;
            fixedHeadingDeg = Double.NaN;
            kicker.setPower(0);
            transfer.setPower(0);
            turret.stopShooter();
            gate.setPosition(T.GATE_CLOSED);
        }

        /** End of OpMode: every output stops even if one device throws. */
        void shutdown() { RobotShutdown.stopAll(this::stopAll, turret::stop); }

        /** Once per loop, before the step: turret encoder, homing and flywheel velocity reads. */
        void readSensors(double now) { turret.update(now); }

        /**
         * Once per loop, after the step. pose: this loop's Pinpoint sample (meters, Pedro frame, CCW).
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

            // ---- hood + flywheel from the measured shot table (range from the turret pivot) ----
            ShotProfile.Row row = ShootingConfiguration.PROFILE.sample(dist * METERS_PER_INCH);
            hoodTargetDeg = hoodAuto && row != null ? row.hoodDeg : hoodManualDeg;
            turret.setPitchAngleDegrees(hoodTargetDeg);
            rpmTarget = rpmAuto ? (row == null ? 0 : row.rpm) : ShooterSpeed.clampFlywheelRpm(rpmManual);
            if (spinning && rpmTarget > 0) turret.setShooterRpm(rpmTarget);
            else turret.stopShooter();

            // ---- shot sequence: open gate -> both transfer stages feed -> stop -> close gate ----
            // Stage 1 also runs whenever the intake is in (setIntake); stage 2 (kicker) only feeds shots.
            double t = clock.seconds();
            switch (shootState) {
                case 1:
                    gate.setPosition(T.GATE_OPEN);
                    if (t - shootT >= T.GATE_OPEN_DELAY_S) {   // gate is open: run both transfers
                        kicker.setPower(T.KICKER_POWER);
                        transfer.setPower(T.TRANSFER_POWER);
                        shootState = 2;
                        shootT = t;
                    }
                    break;
                case 2:
                    if (t - shootT >= T.FEED_PER_BALL_S * shootBalls) {
                        kicker.setPower(0);
                        transfer.setPower(stage1IdlePower()); // back to intake-only loading
                        shootState = 3;
                        shootT = t;
                    }
                    break;
                case 3:
                    if (t - shootT >= T.GATE_CLOSE_DELAY_S) {
                        gate.setPosition(T.GATE_CLOSED);
                        shootState = 0;
                    }
                    break;
                default:
                    break;
            }
        }

        void addTelemetry(Telemetry t) {
            turret.addDriverTelemetry(t);
            t.addData("Turret target heading (deg CW)", "%.1f | %s", turret.getTargetRobotHeadingDegrees(),
                    !turret.isHomed() ? "NOT HOMED" : turretReady() ? "ON TARGET" : tracking && !poseFresh ? "POSE STALE" : "MOVING");
            t.addData("Aim range (in)", "%.1f%s", dist, aimTarget == null ? " (fallback)" : "");
            t.addData("Hood travel (deg) / flywheel target (RPM)", "%.1f / %.0f", hoodTargetDeg, rpmTarget);
            t.addData("Shot", "ready %s | state %d%s", ready(), shootState, shotNote.isEmpty() ? "" : " | " + shotNote);
        }
    }


    // =====================================================================================
    //  OPMODE
    // =====================================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        RobotLoop loop = new RobotLoop(hardwareMap);   // bulk-cached hub reads, cleared once per loop
        drive = new PedroDrive(hardwareMap);   // drive motors + Pinpoint + Pedro 3 follower
        follower = drive.follower();
        cfg = drive.config();
        try {
            robot = new Robot(hardwareMap);    // starts turret homing
            drive.setStartPose(P(60.154, 7.576, -179.2));   // box center (60.15, 7.58) + pod offsets
            boolean lastMirrored = mirrored;
            while (opModeInInit()) {
                loop.begin();
                drive.readSensors();   // applies the start pose once Pinpoint reports READY
                double t = now();   // after the Pinpoint read: pose freshness needs t >= sample time
                robot.readSensors(t);
                robot.apply(t, drive.sample());   // turret homing only; nothing aims or spins before START
                telemetry.addLine("BioBuzz auto ready - keep the robot still during INIT (Pinpoint + turret homing)");
                telemetry.addData("Pose ready", drive.ready(now()));
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                telemetry.update();
            }
            if (isStopRequested()) return;

            final int STEPS = 6;
            int stage = 0;               // steps started so far; running step index = stage - 1
            boolean running = false;
            boolean first = false;
            boolean poseSeen = false;

            while (opModeIsActive() && (stage < STEPS || running)) {
                loop.begin();
                drive.readSensors();
                double t = now();   // after the Pinpoint read: pose freshness needs t >= sample time
                robot.readSensors(t);
                if (!poseSeen) poseSeen = drive.ready(now());

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
                telemetry.addData("step", "%d / %d%s", Math.min(stage, STEPS), STEPS, poseSeen ? "" : " (waiting for Pinpoint pose)");
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                loop.addRateTelemetry(telemetry);
                telemetry.update();
            }
            telemetry.addLine("Auto complete");
            telemetry.update();
        } finally {
            RobotShutdown.stopAll(drive::stop, robot == null ? null : robot::shutdown);
        }
    }

    /** One planner step. first = true on the first call. Returns true when the step is finished. */
    private boolean runStep(int i, boolean first) {
        switch (i) {

            // step 1 - ACTION VOLLEY (AIM + SPIN + SHOOT): 4 ball(s) at aim point 1
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
                        robot.startShoot(4);
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

            // step 3 - DRIVE to (13.5, 8.1) @ -179.2 deg
            case 2: {
                if (first) {
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(60.15, 7.58, 0), P(13.47, 8.11, 0)).constant(H(-179.2)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
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

            // step 5 - DRIVE to (58.5, 21.7) @ -179.2 deg
            case 4: {
                if (first) {
                    Arrays.fill(cbFired, false);
                    follower.follow(limits(Paths.line(P(13.47, 8.11, 0), P(58.46, 21.71, 0)).constant(H(-179.2)),
                            1, 0, T.MAX_ACCEL_IN_PER_S2, T.MAX_DECEL_IN_PER_S2));   // speed fraction, max vel (in/s), accel, decel (in/s^2); 0 = no limit
                }
                boolean done = !first && (!follower.following() || (T.STEP_TIMEOUT_S > 0 && stepTimer.seconds() > T.STEP_TIMEOUT_S));
                return done;
            }

            // step 6 - ACTION VOLLEY (AIM + SPIN + SHOOT): 4 ball(s) at aim point 1
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
                        robot.startShoot(4);
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
