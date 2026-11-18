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
 * from the repo's PedroDrive, so its PedroConfiguration tuning applies; the mechanisms (intake, flywheel,
 * hood, turret, gate, kicker, transfer) are the self-contained Robot class below, configured in class T.
 *
 * - Coordinates: Pedro field inches, bottom-left origin, +x right, +y up. Headings in degrees, CCW positive.
 * - Robot placed as a 23 x 12 in box, center (60.4, 7.29), heading 0 deg.
 * - Start pose (tracked point = box center + pod offsets X 0 fwd, Y 0 reversed): (60.396, 7.291).
 * - Path limits: speed is a fraction of PedroConfiguration.MAX_FORWARD_VELOCITY_IN_PER_SEC; max velocity,
 *   acceleration and deceleration are Foresight constraints for that one path (0 = no limit).
 *
 * HOW IT RUNS: ONE while (opModeIsActive()) loop: drive.readSensors() (the single Pinpoint read), the
 * current step, loop-code blocks, then drive.update(). Steps wait for a fresh, referenced Pinpoint pose.
 * In your own code use: follower, drive, robot, telemetry, stepTimer, P(x, y, deg), H(deg), progress().
 */

@Autonomous(name = "BioBuzzAuto", group = "Auto")
public class testdrive extends LinearOpMode {

    // =====================================================================================
    //  TUNING - change here, or live from FTC Dashboard if @Config is enabled
    // =====================================================================================
    public static class T {
        // ---- Hardware names (must match the robot config) ----
        public static String INTAKE_NAME = "intake";
        public static String TRANSFER_NAME = "transfer";
        public static String KICKER_NAME = "kicker";
        public static String SHOOTER_NAME = "shooter";
        public static String TURRET1_NAME = "turret_servo_1";
        public static String TURRET2_NAME = "turret_servo_2";
        public static String GATE_NAME = "gate";
        public static String HOOD_NAME = "hood";
        // ---- Motor directions ----
        public static boolean INTAKE_REVERSED = false;
        public static boolean TRANSFER_REVERSED = true;
        public static boolean KICKER_REVERSED = false;
        public static boolean SHOOTER_REVERSED = false;
        // ---- Intake ----
        public static double INTAKE_POWER = 1;
        // ---- Flywheel (velocity PIDF + distance model) ----
        public static double TICKS_PER_REV = 28;
        public static double SHOOTER_KP = 16;
        public static double SHOOTER_KI = 2;
        public static double SHOOTER_KD = 0;
        public static double SHOOTER_KF = 12;
        public static double RPM_SLOPE = 6.22;
        public static double RPM_INTERCEPT = 2516;
        public static double RPM_MIN = 0;
        public static double RPM_MAX = 6000;
        public static double RPM_TOLERANCE = 50;
        public static double READY_TIMEOUT_S = 3;
        // ---- Hood (distance model) ----
        public static double HOOD_SLOPE = -0.00309;
        public static double HOOD_INTERCEPT = 0.652;
        public static double HOOD_MIN = 0.5;
        public static double HOOD_MAX = 1;
        // ---- Turret ----
        public static double TURRET_INIT_POS = 0.5;
        public static double TURRET_ZERO_POS = 0.25;
        public static double TURRET_RANGE_DEG = 360;
        public static double TURRET_MIN = 0;
        public static double TURRET_MAX = 1;
        public static double TURRET_SEC_PER_FULL_TRAVEL = 0.9;
        public static double TURRET_SETTLE_PAD_S = 0.1;
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
    //  ROBOT SUBSYSTEMS - intake, flywheel, hood, turret, gate, kicker + transfer.
    //  Everything is in this file. update() must be called every loop (the OpMode does it).
    // =====================================================================================
    static class Robot {
        final DcMotorEx intake, transfer, kicker, shooter;
        final Servo turret1, turret2, gate, hood;
        final ElapsedTime clock = new ElapsedTime();

        int intakeMode = 0;                  // 1 in, -1 out, 0 off

        boolean spinning = false;
        boolean rpmAuto = true;
        double rpmManual = 0;
        double rpmTarget = 0;

        Pose aimTarget = null;               // point the turret tracks / distance is measured to
        boolean tracking = false;
        double turretPos = T.TURRET_INIT_POS;
        double turretSettleAt = 0;
        double dist = T.FALLBACK_DIST_IN;

        boolean hoodAuto = true;
        double hoodManual = T.HOOD_MIN;

        int shootState = 0;                  // 0 idle, 1 gate opening, 2 feeding, 3 closing
        int shootBalls = 0;
        double shootT = 0;

        Robot(HardwareMap hw) {
            intake = hw.get(DcMotorEx.class, T.INTAKE_NAME);
            transfer = hw.get(DcMotorEx.class, T.TRANSFER_NAME);
            kicker = hw.get(DcMotorEx.class, T.KICKER_NAME);
            shooter = hw.get(DcMotorEx.class, T.SHOOTER_NAME);
            turret1 = hw.get(Servo.class, T.TURRET1_NAME);
            turret2 = hw.get(Servo.class, T.TURRET2_NAME);
            gate = hw.get(Servo.class, T.GATE_NAME);
            hood = hw.get(Servo.class, T.HOOD_NAME);

            setup(intake, T.INTAKE_REVERSED);
            setup(transfer, T.TRANSFER_REVERSED);
            setup(kicker, T.KICKER_REVERSED);
            setup(shooter, T.SHOOTER_REVERSED);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyPidf();

            gate.setPosition(T.GATE_CLOSED);
            commandTurret(T.TURRET_INIT_POS);
            hood.setPosition(clamp(T.HOOD_MIN, 0, 1));
        }

        private static void setup(DcMotorEx m, boolean reversed) {
            m.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        void applyPidf() {
            shooter.setVelocityPIDFCoefficients(T.SHOOTER_KP, T.SHOOTER_KI, T.SHOOTER_KD, T.SHOOTER_KF);
        }

        // ---- commands (call these from code steps / loop code too) ----
        void setIntake(int mode) { intakeMode = mode; intake.setPower(mode * T.INTAKE_POWER); }
        void spin(boolean on) { spinning = on; }
        void setRpmAuto() { rpmAuto = true; }
        void setRpmManual(double rpm) { rpmAuto = false; rpmManual = rpm; }
        void setHoodAuto() { hoodAuto = true; }
        void setHoodManual(double pos) { hoodAuto = false; hoodManual = clamp(pos, 0, 1); }
        void aimAt(Pose target) { aimTarget = target; }
        void setTracking(boolean on) { tracking = on; }
        void setTurretManual(double pos) { tracking = false; commandTurret(pos); }
        void gateOpen(boolean open) { gate.setPosition(open ? T.GATE_OPEN : T.GATE_CLOSED); }

        void startShoot(int balls) {
            shootBalls = Math.max(1, balls);
            shootState = 1;
            shootT = clock.seconds();
        }
        boolean isShooting() { return shootState != 0; }

        boolean turretReady() { return clock.seconds() >= turretSettleAt; }

        /** Flywheel at target speed (and turret settled when tracking). */
        boolean ready() {
            if (!spinning || rpmTarget <= 0) return false;
            double rpm = shooter.getVelocity() * 60.0 / T.TICKS_PER_REV;
            return Math.abs(rpm - rpmTarget) <= T.RPM_TOLERANCE && (!tracking || turretReady());
        }

        void stopAll() {
            setIntake(0);
            spinning = false;
            tracking = false;
            shootState = 0;
            kicker.setPower(0);
            transfer.setPower(0);
            shooter.setPower(0);
            gate.setPosition(T.GATE_CLOSED);
        }

        void commandTurret(double pos) {
            while (pos < 0) pos += 1;
            while (pos >= 1) pos -= 1;
            pos = clamp(pos, T.TURRET_MIN, T.TURRET_MAX);
            if (Math.abs(pos - turretPos) > 1e-4) {
                turretSettleAt = clock.seconds()
                        + Math.abs(pos - turretPos) * T.TURRET_SEC_PER_FULL_TRAVEL + T.TURRET_SETTLE_PAD_S;
            }
            turretPos = pos;
            turret1.setPosition(pos);
            turret2.setPosition(pos);
        }

        /** Call every loop with the follower pose. */
        void update(Pose pose) {
            double now = clock.seconds();

            // ---- aim: same math as the TeleOp (servo 0..1 spans TURRET_RANGE_DEG) ----
            if (aimTarget != null) {
                double dx = aimTarget.x() - pose.x(), dy = aimTarget.y() - pose.y();
                dist = Math.hypot(dx, dy);
                if (tracking) {
                    double a = wrap(Math.atan2(dy, dx) - pose.heading());
                    commandTurret(T.TURRET_ZERO_POS - a / Math.toRadians(T.TURRET_RANGE_DEG));
                }
            }

            // ---- flywheel + hood from distance ----
            rpmTarget = rpmAuto ? clamp(T.RPM_SLOPE * dist + T.RPM_INTERCEPT, T.RPM_MIN, T.RPM_MAX)
                    : clamp(rpmManual, T.RPM_MIN, T.RPM_MAX);
            if (spinning) shooter.setVelocity(rpmTarget * T.TICKS_PER_REV / 60.0);
            else shooter.setPower(0);

            double h = hoodAuto ? clamp(T.HOOD_SLOPE * dist + T.HOOD_INTERCEPT, T.HOOD_MIN, T.HOOD_MAX) : hoodManual;
            hood.setPosition(h);

            // ---- shot sequence: open gate -> feed kicker + transfer -> stop -> close gate ----
            switch (shootState) {
                case 1:
                    gate.setPosition(T.GATE_OPEN);
                    if (now - shootT >= T.GATE_OPEN_DELAY_S) {
                        kicker.setPower(T.KICKER_POWER);
                        transfer.setPower(T.TRANSFER_POWER);
                        shootState = 2;
                        shootT = now;
                    }
                    break;
                case 2:
                    if (now - shootT >= T.FEED_PER_BALL_S * shootBalls) {
                        kicker.setPower(0);
                        transfer.setPower(0);
                        shootState = 3;
                        shootT = now;
                    }
                    break;
                case 3:
                    if (now - shootT >= T.GATE_CLOSE_DELAY_S) {
                        gate.setPosition(T.GATE_CLOSED);
                        shootState = 0;
                    }
                    break;
                default:
                    break;
            }
        }

        void addTelemetry(Telemetry t) {
            t.addData("flywheel rpm", "%.0f / %.0f", shooter.getVelocity() * 60.0 / T.TICKS_PER_REV, rpmTarget);
            t.addData("ready", ready());
            t.addData("aim dist", "%.1f in", dist);
            t.addData("turret pos", "%.3f", turretPos);
            t.addData("shot state", shootState);
        }
    }


    // =====================================================================================
    //  OPMODE
    // =====================================================================================
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new PedroDrive(hardwareMap);   // drive motors + Pinpoint + Pedro 3 follower
        follower = drive.follower();
        cfg = drive.config();
        robot = new Robot(hardwareMap);
        try {
            drive.setStartPose(P(60.396, 7.291, 0));   // box center (60.4, 7.29) + pod offsets
            boolean lastMirrored = mirrored;
            while (opModeInInit()) {
                drive.readSensors();   // applies the start pose once Pinpoint reports READY
                telemetry.addLine("BioBuzz auto ready - keep the robot still during INIT");
                telemetry.addData("Pose ready", drive.ready(now()));
                drive.addTelemetry(telemetry);
                telemetry.update();
            }
            if (isStopRequested()) return;

            final int STEPS = 4;
            int stage = 0;               // steps started so far; running step index = stage - 1
            boolean running = false;
            boolean first = false;
            boolean poseSeen = false;

            while (opModeIsActive() && (stage < STEPS || running)) {
                drive.readSensors();
                robot.update(follower.pose());
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

                drive.update();
                telemetry.addData("step", "%d / %d%s", Math.min(stage, STEPS), STEPS, poseSeen ? "" : " (waiting for Pinpoint pose)");
                drive.addTelemetry(telemetry);
                robot.addTelemetry(telemetry);
                telemetry.update();
            }
            telemetry.addLine("Auto complete");
            telemetry.update();
        } finally {
            drive.stop();
            robot.stopAll();
        }
    }

    /** One planner step. first = true on the first call. Returns true when the step is finished. */
    private boolean runStep(int i, boolean first) {
        switch (i) {

            // step 1 - ACTION INTAKE: in · 2s
            case 0: {
                if (first) robot.setIntake(1);
                boolean done = stepTimer.seconds() >= 2;
                return done;
            }

            // step 2 - ACTION INTAKE: off
            case 1: {
                if (first) robot.setIntake(0);
                return true;
            }

            // step 3 - ACTION VOLLEY (AIM + SPIN + SHOOT): 1 ball(s) at aim point 1
            case 2: {
                if (first) {
                    robot.aimAt(P(T.GOAL1_X, T.GOAL1_Y, 0));
                    robot.setTracking(true);
                    robot.spin(true);
                    robot.setRpmAuto();
                    robot.setHoodAuto();
                    robot.update(follower.pose());
                }
                if (phase == 0) {
                    if (robot.ready() || stepTimer.seconds() > T.READY_TIMEOUT_S) {
                        robot.startShoot(1);
                        phase = 1;
                    }
                    return false;
                }
                if (!robot.isShooting()) {
                    return true;
                }
                return false;
            }

            // step 4 - ACTION STOP ALL: intake, flywheel, feed off
            case 3: {
                robot.stopAll();
                return true;
            }

            default:
                return true;
        }
    }
}
