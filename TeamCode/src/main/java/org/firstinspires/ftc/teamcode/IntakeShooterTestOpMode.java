package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Combined intake/index/shooter bench test with no drive, odometry, camera or depollinator. */
@TeleOp(name = "Test: Intake + Index + Shooter", group = "Test")
@Config
public class IntakeShooterTestOpMode extends LinearOpMode {
    // Bench-only settings. Dashboard changes persist in memory until the RC app restarts.
    public static volatile double P = ShooterVelocityTuning.INITIAL_P;
    public static volatile double I = ShooterVelocityTuning.INITIAL_I;
    public static volatile double D = ShooterVelocityTuning.INITIAL_D;
    public static volatile double F_SCALE = ShooterVelocityTuning.INITIAL_F_SCALE;
    public static volatile double TARGET_RPM = TurretSubsystem.SHOOTER_DEFAULT_TARGET_RPM;
    // Fusion controller (GP2 B): beyond 80 ticks/s error, full power when slow and coast when fast;
    // inside that band, kS + kV * target + kP * error.
    // Motor power at 12 V; kV and kP are per motor tick/s. Untuned starting estimates.
    public static volatile double FUSION_KS = ShooterVelocityTuning.INITIAL_FUSION_KS;
    public static volatile double FUSION_KV = ShooterVelocityTuning.INITIAL_FUSION_KV;
    public static volatile double FUSION_KP = ShooterVelocityTuning.INITIAL_FUSION_KP;
    private static final double PITCH_STEP_DEG = 0.5;
    private static final double SHOOTER_RPM_STEP = 100.0;
    private static final double MAX_LOOP_DT_SECONDS = 0.1;
    private static final int DASHBOARD_INTERVAL_MS = 50;
    private static final double TIMING_STEP_SECONDS = 0.05;
    private static final double MAX_TIMING_SECONDS = 5.0;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // No Dashboard service exists in headless JVM tests.
        int previousDashboardIntervalMs = dashboard == null ? 0 : dashboard.getTelemetryTransmissionInterval();
        RobotLoop loop = new RobotLoop(hardwareMap);
        GateSubsystem gate = null;
        IntakeSubsystem intake = null;
        TransferSubsystem transfer = null;
        TurretSubsystem turret = null;
        try {
            if (dashboard != null) dashboard.setTelemetryTransmissionInterval(DASHBOARD_INTERVAL_MS);
            gate = new GateSubsystem(hardwareMap, IntakeShooterTestController.GATE_TO_FEED_DELAY_SECONDS);
            // Bench jam detection requires this motor's own encoder, separate from turret feedback.
            intake = new IntakeSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap);
            turret = new TurretSubsystem(hardwareMap, false);
            IntakeShooterTestController controls = new IntakeShooterTestController();
            double selectedRpm = bounded(TARGET_RPM, 0, 0, TurretSubsystem.SHOOTER_MAX_RPM);
            boolean previousHome = false;

            while (opModeInInit()) {
                loop.begin();
                if (gamepad1.x && !previousHome) turret.setAllServosToZeroPosition();
                previousHome = gamepad1.x;
                turret.update();
                turret.applyYawCommand();
                addControlsTelemetry();
                telemetry.addLine("Turret homes during INIT: home points robot-right (90 deg).");
                telemetry.addLine("After START, release bumpers/triggers to enable motor controls.");
                turret.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                loop.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            boolean previousRpmUp = false, previousRpmDown = false;
            boolean previousHoodUp = false, previousHoodDown = false;
            boolean previousPidUp = false, previousPidDown = false;
            boolean previousFUp = false, previousFDown = false, previousPidReset = false;
            boolean previousIUp = false, previousIDown = false;
            boolean previousControllerToggle = gamepad2.b;
            boolean previousShootingToggle = gamepad2.right_bumper;
            boolean wasFeeding = false;
            double minimumFeedRpm = Double.NaN;
            double recoveryStarted = Double.NaN, lastRecoverySeconds = Double.NaN;
            double previousTime = System.nanoTime() * 1e-9;
            double lastDashboardTime = Double.NEGATIVE_INFINITY;
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(MAX_LOOP_DT_SECONDS, now - previousTime));
                previousTime = now;
                FiringCoordinator firing = controls.firing();
                boolean timing = gamepad2.left_bumper;
                boolean shootingToggle = gamepad2.right_bumper && !previousShootingToggle;
                previousShootingToggle = gamepad2.right_bumper;
                double feedSeconds = firing.feedWindowSeconds;
                double pauseSeconds = firing.recoverySeconds;
                double readySeconds = firing.stableSeconds;
                if (timing) {
                    pauseSeconds = adjustTiming(pauseSeconds, gamepad2.dpad_up && !previousPidUp,
                            gamepad2.dpad_down && !previousPidDown, 0);
                    feedSeconds = adjustTiming(feedSeconds, gamepad2.dpad_right && !previousFUp,
                            gamepad2.dpad_left && !previousFDown, TIMING_STEP_SECONDS);
                    readySeconds = adjustTiming(readySeconds, gamepad2.y && !previousIUp,
                            gamepad2.a && !previousIDown, 0);
                    if (gamepad2.x && !previousPidReset) {
                        feedSeconds = IntakeShooterTestController.FEED_WINDOW_SECONDS;
                        pauseSeconds = ShootingConfiguration.RECOVERY_SECONDS;
                        readySeconds = ShootingConfiguration.READY_STABLE_SECONDS;
                    }
                }
                boolean shootingChanged = controls.configureShooting(now,
                        shootingToggle ? !firing.continuous() : firing.continuous(),
                        feedSeconds, pauseSeconds, readySeconds);
                boolean controllerChanged = gamepad2.b && !previousControllerToggle;
                previousControllerToggle = gamepad2.b;
                if (controllerChanged) {
                    turret.setShooterController(turret.getShooterController() == TurretSubsystem.ShooterController.PIDF
                            ? TurretSubsystem.ShooterController.BANG_BANG_FUSION : TurretSubsystem.ShooterController.PIDF);
                    minimumFeedRpm = Double.NaN;
                    recoveryStarted = lastRecoverySeconds = Double.NaN;
                    wasFeeding = false;
                }
                double deltaP = (gamepad2.dpad_up && !previousPidUp ? 1.0 : 0)
                        - (gamepad2.dpad_down && !previousPidDown ? 1.0 : 0);
                double deltaF = (gamepad2.dpad_right && !previousFUp ? .05 : 0)
                        - (gamepad2.dpad_left && !previousFDown ? .05 : 0);
                double deltaI = (gamepad2.y && !previousIUp ? ShooterVelocityTuning.I_STEP : 0)
                        - (gamepad2.a && !previousIDown ? ShooterVelocityTuning.I_STEP : 0);
                if (!timing && gamepad2.x && !previousPidReset) {
                    P = ShooterVelocityTuning.INITIAL_P;
                    I = ShooterVelocityTuning.INITIAL_I;
                    D = ShooterVelocityTuning.INITIAL_D;
                    F_SCALE = ShooterVelocityTuning.INITIAL_F_SCALE;
                    FUSION_KS = ShooterVelocityTuning.INITIAL_FUSION_KS;
                    FUSION_KV = ShooterVelocityTuning.INITIAL_FUSION_KV;
                    FUSION_KP = ShooterVelocityTuning.INITIAL_FUSION_KP;
                } else if (!timing) {
                    if (deltaP != 0) P = bounded(P + deltaP, turret.getShooterP(), 0, 100);
                    if (deltaI != 0) I = bounded(I + deltaI, turret.getShooterI(), 0, ShooterVelocityTuning.MAX_I);
                    if (deltaF != 0) F_SCALE = bounded(F_SCALE + deltaF, turret.getShooterFScale(), .5, 1.5);
                }
                turret.setShooterVelocityTuning(
                        bounded(P, turret.getShooterP(), 0, 100),
                        bounded(I, turret.getShooterI(), 0, ShooterVelocityTuning.MAX_I),
                        bounded(D, turret.getShooterD(), 0, 100),
                        bounded(F_SCALE, turret.getShooterFScale(), .5, 1.5));
                turret.setShooterFusionTuning(
                        bounded(FUSION_KS, turret.getShooterFusionKs(), 0, ShooterVelocityTuning.MAX_FUSION_KS),
                        bounded(FUSION_KV, turret.getShooterFusionKv(), 0, ShooterVelocityTuning.MAX_FUSION_KV),
                        bounded(FUSION_KP, turret.getShooterFusionKp(), 0, ShooterVelocityTuning.MAX_FUSION_KP));
                previousPidUp = gamepad2.dpad_up; previousPidDown = gamepad2.dpad_down;
                previousFUp = gamepad2.dpad_right; previousFDown = gamepad2.dpad_left;
                previousPidReset = gamepad2.x;
                previousIUp = gamepad2.y; previousIDown = gamepad2.a;
                boolean load = gamepad1.right_bumper;
                boolean reverse = gamepad1.left_bumper;
                boolean spin = gamepad1.right_trigger > GateSubsystem.TRIGGER_DEADBAND;
                boolean feed = gamepad1.left_trigger > GateSubsystem.TRIGGER_DEADBAND;
                boolean rehome = gamepad1.x && !previousHome && !gamepad1.b
                        && !load && !reverse && !spin && !feed;
                previousHome = gamepad1.x;
                // Close the gate and re-arm after a switch; a held trigger cannot resume feeding.
                boolean stop = gamepad1.b || rehome || controllerChanged || shootingChanged;

                if (rehome) turret.setAllServosToZeroPosition();
                boolean rpmChanged = false, hoodChanged = false;
                if (!stop) {
                    double rpmDelta = (gamepad1.y && !previousRpmUp ? SHOOTER_RPM_STEP : 0)
                            - (gamepad1.a && !previousRpmDown ? SHOOTER_RPM_STEP : 0);
                    if (rpmDelta != 0) TARGET_RPM = bounded(TARGET_RPM + rpmDelta, 0, 0, TurretSubsystem.SHOOTER_MAX_RPM);
                    double nextRpm = bounded(TARGET_RPM, 0, 0, TurretSubsystem.SHOOTER_MAX_RPM);
                    rpmChanged = nextRpm != selectedRpm;
                    selectedRpm = nextRpm;
                    double hoodDelta = (gamepad1.dpad_up && !previousHoodUp ? PITCH_STEP_DEG : 0)
                            - (gamepad1.dpad_down && !previousHoodDown ? PITCH_STEP_DEG : 0);
                    hoodChanged = hoodDelta != 0;
                    if (hoodChanged) turret.adjustPitchAngleDegrees(hoodDelta);
                }
                previousRpmUp = gamepad1.y;
                previousRpmDown = gamepad1.a;
                previousHoodUp = gamepad1.dpad_up;
                previousHoodDown = gamepad1.dpad_down;

                turret.update();
                if (!stop) turret.updateManualYaw(gamepad1.right_stick_x, dt, false);
                turret.applyYawCommand();
                boolean yawInput = Math.abs(gamepad1.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND;
                // Compare cached measured speed against this loop's selected target.
                boolean atSpeed = selectedRpm > 0 && Math.abs(turret.getShooterCurrentRpm() - selectedRpm)
                        <= TurretSubsystem.SHOOTER_READY_TOLERANCE_RPM;
                if (stop || !spin || rpmChanged) recoveryStarted = Double.NaN;
                else if (!atSpeed && Double.isNaN(recoveryStarted)) recoveryStarted = now;
                else if (atSpeed && !Double.isNaN(recoveryStarted)) {
                    lastRecoverySeconds = now - recoveryStarted;
                    recoveryStarted = Double.NaN;
                }
                double yawErrorDeg = turret.getTargetYawDegrees() - turret.getEncoderAngleDegrees();
                boolean yawSettled = IntakeShooterTestController.isYawSettled(yawErrorDeg,
                        turret.getEncoderVelocityDegPerSec());
                boolean hoodSettled = turret.isHoodTracking();
                boolean ready = atSpeed && yawSettled
                        && hoodSettled && !yawInput && !rpmChanged && !hoodChanged;
                controls.update(now, load, reverse, spin && selectedRpm > 0, feed, stop, turret.isHomed(), ready);
                if (controls.spinFlywheel() && selectedRpm > 0) turret.setShooterRpm(selectedRpm);
                else turret.stopShooter();

                gate.setOpen(firing.gateOpen());
                boolean feeding = controls.feedTransfer(gate.isOpenForFeeding());
                if (feeding) minimumFeedRpm = !wasFeeding ? turret.getShooterCurrentRpm()
                        : Math.min(minimumFeedRpm, turret.getShooterCurrentRpm());
                wasFeeding = feeding;
                transfer.update(now, feeding, !controls.loadIndex() || firing.gateOpen(), controls.feedStage1());
                double intakeVelocityTicksPerSecond = intake.getVelocityTicksPerSecond();
                intake.setIntakePower(controls.updateIntakePower(now, transfer.isFull(), intakeVelocityTicksPerSecond));

                // Put the exact blocker first so it remains visible above the hardware diagnostics.
                String feedStatus;
                if (stop) feedStatus = "STOP requested";
                else if (!controls.armed()) feedStatus = "Release B, bumpers and triggers to enable";
                else if (reverse) feedStatus = "Release LB: reverse intake inhibits feed";
                else if (!feed) feedStatus = "Hold GP1 LT with RT to request feed";
                else if (!spin) feedStatus = "Hold GP1 RT to spin the flywheel";
                else if (selectedRpm <= 0) feedStatus = "Selected RPM is zero: tap Y to increase";
                else if (firing.feeding()) feedStatus = (firing.continuous() ? "CONTINUOUS" : "STAGGERED")
                        + (controls.feedStage1() ? " FEED: both stages forward" : " FEED: stage 2 launching; stage 1 waits for midpoint")
                        + "; release either trigger to stop";
                else if (!turret.isHomed()) feedStatus = "Home required: " + turret.getHomingStatus();
                else if (!atSpeed) feedStatus = "Flywheel outside RPM tolerance: check actual/target and encoder cable";
                else if (yawInput) feedStatus = "Release right stick to settle turret";
                else if (!yawSettled) feedStatus = "Turret outside 4 deg / 3 deg/s bench limits";
                else if (!hoodSettled || hoodChanged) feedStatus = "Waiting for hood travel";
                else if (rpmChanged) feedStatus = "RPM target changed: requalifying readiness";
                else if (firing.state() == FiringCoordinator.State.PREPARING)
                    feedStatus = "Checks passing: qualifying stable readiness";
                else feedStatus = firing.reason();
                transfer.addStatusTelemetry(telemetry);
                telemetry.addData("FEED STATUS", feedStatus);
                telemetry.addData("Shooting mode", firing.continuous() ? "CONTINUOUS" : "STAGGERED");
                telemetry.addData("Shooting timing (s)", "feed %.2f | recovery pause %.2f | ready delay %.2f",
                        firing.feedWindowSeconds, firing.recoverySeconds, firing.stableSeconds);
                telemetry.addData("GP1 triggers / LB", "RT %.2f | LT %.2f | LB %s",
                        gamepad1.right_trigger, gamepad1.left_trigger, reverse);
                telemetry.addData("Feed flywheel actual / selected (RPM)", "%.0f / %.0f",
                        turret.getShooterCurrentRpm(), selectedRpm);
                telemetry.addData("Lowest flywheel RPM in last/current feed", minimumFeedRpm);
                telemetry.addData("Last return to RPM tolerance (s)", lastRecoverySeconds);
                if (!Double.isNaN(recoveryStarted)) telemetry.addData("RPM recovery elapsed (s)", now - recoveryStarted);
                telemetry.addData("Feed yaw error / rate (deg, deg/s)", "%.2f / %.2f",
                        yawErrorDeg, turret.getEncoderVelocityDegPerSec());
                telemetry.addData("Motor controls", controls.armed() ? "ENABLED"
                        : "STOPPED: release B, bumpers and triggers to enable");
                telemetry.addData("Selected flywheel (RPM)", "%.0f (hold RT to spin)", selectedRpm);
                telemetry.addData("Feed checks", "home %s | RPM %s | yaw %s | hood %s",
                        turret.isHomed(), atSpeed, yawSettled, hoodSettled);
                telemetry.addData("Feed state", "%s | %s", firing.state(),
                        firing.feeding() ? (firing.continuous() ? "Continuous" : "Staggered") + " two-stage forward feed" : firing.reason());
                telemetry.addData("Staggered shooting (s)", "gate delay %.2f | stage 2 feed %.2f | recovery %.2f",
                        firing.gateTravelSeconds, firing.feedWindowSeconds, firing.recoverySeconds);
                telemetry.addData("Staggered S1 start / gate open window (s)", "%.2f / %.2f",
                        (firing.gateTravelSeconds + firing.feedWindowSeconds) / 2.0,
                        firing.gateTravelSeconds + firing.feedWindowSeconds);
                telemetry.addData("Feed starts (not ball count)", firing.feedAttempts());
                telemetry.addData("Feed motors", feeding ? "RUNNING" : "STOPPED");
                telemetry.addData("Flywheel regulation", turret.getShooterController());
                addControlsTelemetry();
                telemetry.addData("Intake velocity (ticks/s)", intakeVelocityTicksPerSecond);
                telemetry.addData("Intake jam recovery", controls.isIntakeClearing() ? "OSCILLATING (0.5 s)" : "Monitoring");
                intake.addTelemetry(telemetry);
                transfer.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                turret.addTelemetry(telemetry);
                loop.addTelemetry(telemetry);
                if (dashboard != null && now - lastDashboardTime >= DASHBOARD_INTERVAL_MS / 1000.0) {
                    lastDashboardTime = now;
                    TelemetryPacket packet = new TelemetryPacket();
                    // Numeric fields can be selected directly in Dashboard's graph widget.
                    packet.put("Selected RPM", selectedRpm);
                    packet.put("Target RPM", turret.getTargetShooterRpm());
                    packet.put("Measured RPM", turret.getShooterCurrentRpm());
                    packet.put("Error RPM", turret.getTargetShooterRpm() - turret.getShooterCurrentRpm());
                    if (Double.isFinite(turret.getShooterBatteryVolts()))
                        packet.put("Battery V (5 Hz)", turret.getShooterBatteryVolts());
                    if (Double.isFinite(turret.getShooterCurrentAmps()))
                        packet.put("Current A (5 Hz)", turret.getShooterCurrentAmps());
                    packet.put("P", turret.getShooterP());
                    packet.put("I", turret.getShooterI());
                    packet.put("D", turret.getShooterD());
                    packet.put("F scale", turret.getShooterFScale());
                    packet.put("Applied F (PIDF only)", turret.getShooterAppliedF());
                    packet.put("Error ticks/s", ShooterSpeed.motorTicksPerSecond(turret.getTargetShooterRpm())
                            - turret.getShooterTicksPerSecond());
                    packet.put("Fusion power", turret.getShooterFusionPower());
                    packet.put("Fusion bang-bang (0/1)", turret.isShooterFusionBangBang() ? 1 : 0);
                    packet.put("Fusion kS", turret.getShooterFusionKs());
                    packet.put("Fusion kV", turret.getShooterFusionKv());
                    packet.put("Fusion kP", turret.getShooterFusionKp());
                    packet.put("Feeding (0/1)", feeding ? 1 : 0);
                    packet.put("At speed (0/1)", atSpeed ? 1 : 0);
                    if (Double.isFinite(minimumFeedRpm)) packet.put("Minimum feed RPM", minimumFeedRpm);
                    if (Double.isFinite(lastRecoverySeconds)) packet.put("Last return to tolerance s", lastRecoverySeconds);
                    packet.put("Controller", turret.getShooterController().name());
                    packet.put("Feed state", firing.state().name());
                    packet.put("Feed status", feedStatus);
                    dashboard.sendTelemetryPacket(packet);
                }
                telemetry.update();
            }
        } finally {
            try {
                RobotShutdown.stopAll(intake == null ? null : intake::stop,
                        transfer == null ? null : transfer::stop,
                        gate == null ? null : gate::close,
                        turret == null ? null : turret::stop);
            } finally {
                if (dashboard != null) dashboard.setTelemetryTransmissionInterval(previousDashboardIntervalMs);
            }
        }
    }

    private static double bounded(double value, double fallback, double min, double max) {
        return Double.isFinite(value) ? Math.max(min, Math.min(max, value)) : fallback;
    }

    private static double adjustTiming(double seconds, boolean up, boolean down, double minimum) {
        // Round to avoid accumulating floating-point drift from repeated 50 ms steps.
        return Math.max(minimum, Math.min(MAX_TIMING_SECONDS,
                Math.round((seconds + ((up ? 1 : 0) - (down ? 1 : 0)) * TIMING_STEP_SECONDS) * 100) / 100.0));
    }

    private void addControlsTelemetry() {
        telemetry.addLine("Dashboard: 192.168.43.1:8080/dash | Config: IntakeShooterTestOpMode");
        telemetry.addLine("Live P/I/D/F_SCALE/TARGET_RPM and FUSION_KS/KV/KP; applied gains shown below; GP2 X resets gains unless LB held");
        telemetry.addLine("Automatic intake/index after arming: LOW forward, 0.2 s HIGH stop | RB forces forward / LB reverse / Both stop");
        telemetry.addLine("Hold RT = Flywheel | RT + LT: Continuous = Immediate Full Feed; Staggered = Readiness + Timed Cycle");
        telemetry.addLine("GP2 RB = Continuous/Staggered (default); changes require GP1 controls released");
        telemetry.addLine("Hold GP2 LB: Dpad U/D = Recovery Pause, R/L = Feed Duration, Y/A = Ready Delay +/-0.05 s; X = Timing Reset");
        telemetry.addLine("GP2 PID tuning (PIDF only): Dpad U/D = P +/-1 | R/L = F scale +/-0.05 | Y/A = I +/-0.05 | X = Reset All Gains");
        telemetry.addLine("GP2 B = Toggle PIDF / Fusion (Full Power when 80+ ticks/s slow, Coast when 80+ fast, else kS/kV/kP Feedforward); release GP1 motor controls to re-arm");
        telemetry.addLine("Y/A = RPM +/-100 | Dpad Up = Lower Hood, Down = Raise Hood (0.5 deg) | RS-X = Turret");
        telemetry.addLine("B = Stop Motors + Close Gate (release controls to resume) | X = Rehome with motor controls released");
    }
}
