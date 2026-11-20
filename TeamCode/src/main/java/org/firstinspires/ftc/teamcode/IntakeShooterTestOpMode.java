package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Combined intake/index/shooter bench test with no drive, odometry, camera or depollinator.
 * Shooting follows the competition TeleOp (see {@link IntakeShooterTestController}): continuous feed, RT + LT
 * automatic shots once ready, GP2 RT manual feed, GP2 LT inhibit, fusion flywheel regulation, GP1 rumble and the
 * shooter/intake status lights. Bench-only: no flywheel idle (it runs only while RT is held), transfer stage 1
 * starts {@link IntakeShooterTestController#GATE_OPEN_TO_TRANSFER_SECONDS} after the gate-open command, GP1 B
 * also feeds manually and GP1 Back is the stop.
 */
@TeleOp(name = "Test: Intake + Index + Shooter", group = "Test")
@Config
public class IntakeShooterTestOpMode extends LinearOpMode {
    // Bench-only settings. Dashboard changes persist in memory until the RC app restarts.
    public static volatile double P = ShooterVelocityTuning.INITIAL_P;
    public static volatile double I = ShooterVelocityTuning.INITIAL_I;
    public static volatile double D = ShooterVelocityTuning.INITIAL_D;
    public static volatile double F_SCALE = ShooterVelocityTuning.INITIAL_F_SCALE;
    /** RT (selected) RPM, starting at the competition TeleOp's manual RPM. The bench itself has no idle. */
    public static volatile double TARGET_RPM = MecanumTeleOp.FLYWHEEL_IDLE_RPM;
    // Fusion controller (competition default; GP2 B toggles PIDF): beyond 80 ticks/s error, full power when slow
    // and coast when fast; inside that band, kS + kV * target + kP * error.
    // Motor power at 12 V; kV and kP are per motor tick/s. Untuned starting estimates.
    public static volatile double FUSION_KS = ShooterVelocityTuning.INITIAL_FUSION_KS;
    public static volatile double FUSION_KV = ShooterVelocityTuning.INITIAL_FUSION_KV;
    public static volatile double FUSION_KP = ShooterVelocityTuning.INITIAL_FUSION_KP;
    private static final double PITCH_STEP_DEG = 0.5;
    private static final double TRIGGER_DEADBAND = GateSubsystem.TRIGGER_DEADBAND;
    private static final double MAX_LOOP_DT_SECONDS = 0.1;
    private static final int DASHBOARD_INTERVAL_MS = 50;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = dashboardOrNull();
        // No Dashboard service exists in headless JVM tests.
        int previousDashboardIntervalMs = dashboard == null ? 0 : dashboard.getTelemetryTransmissionInterval();
        RobotLoop loop = new RobotLoop(hardwareMap);
        GateSubsystem gate = null;
        IntakeSubsystem intake = null;
        TransferSubsystem transfer = null;
        TurretSubsystem turret = null;
        RobotStatusLights lights = null;
        try {
            if (dashboard != null) dashboard.setTelemetryTransmissionInterval(DASHBOARD_INTERVAL_MS);
            gate = new GateSubsystem(hardwareMap, IntakeShooterTestController.GATE_OPEN_TO_TRANSFER_SECONDS);
            intake = new IntakeSubsystem(hardwareMap);
            transfer = new TransferSubsystem(hardwareMap);
            turret = new TurretSubsystem(hardwareMap, false);
            turret.setShooterController(MecanumTeleOp.FLYWHEEL_CONTROLLER);
            lights = new RobotStatusLights(hardwareMap); // Optional: missing lights are only reported.
            IntakeShooterTestController controls = new IntakeShooterTestController();
            double selectedRpm = IntakeShooterTestController.stepRpm(TARGET_RPM, false, false, false, false);
            boolean previousHome = false;

            while (opModeInInit()) {
                loop.begin();
                if (gamepad1.x && !previousHome) turret.setAllServosToZeroPosition();
                previousHome = gamepad1.x;
                turret.update();
                turret.applyYawCommand();
                lights.showFaultAlert(System.nanoTime() * 1e-9, turret.hasHomingFault());
                addControlsTelemetry();
                telemetry.addLine("Turret homes during INIT: home points robot-right (90 deg).");
                telemetry.addLine("After START, release Back, B, bumpers and triggers to enable motor controls.");
                telemetry.addLine("The flywheel stays off until RT is held (no idle in this test).");
                turret.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                lights.addWarningTelemetry(telemetry);
                loop.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            DriverRumble rumble = new DriverRumble(); // GP1: queue full; RT held until the shot is ready.
            boolean previousRpmUp = false, previousRpmDown = false;
            boolean previousRpmFineUp = false, previousRpmFineDown = false;
            boolean previousHoodUp = false, previousHoodDown = false;
            boolean previousPidUp = false, previousPidDown = false;
            boolean previousFUp = false, previousFDown = false, previousPidReset = false;
            boolean previousIUp = false, previousIDown = false;
            boolean previousControllerToggle = gamepad2.b;
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
                if (gamepad2.x && !previousPidReset) {
                    P = ShooterVelocityTuning.INITIAL_P;
                    I = ShooterVelocityTuning.INITIAL_I;
                    D = ShooterVelocityTuning.INITIAL_D;
                    F_SCALE = ShooterVelocityTuning.INITIAL_F_SCALE;
                    FUSION_KS = ShooterVelocityTuning.INITIAL_FUSION_KS;
                    FUSION_KV = ShooterVelocityTuning.INITIAL_FUSION_KV;
                    FUSION_KP = ShooterVelocityTuning.INITIAL_FUSION_KP;
                } else {
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
                // Competition mapping: bumpers on either gamepad, GP2 RT manual feed, GP2 LT inhibit.
                // Bench-only: GP1 B also feeds manually; GP1 Back is the stop.
                boolean load = gamepad1.right_bumper || gamepad2.right_bumper;
                boolean reverse = gamepad1.left_bumper || gamepad2.left_bumper;
                boolean spin = gamepad1.right_trigger > TRIGGER_DEADBAND;
                boolean feed = gamepad1.left_trigger > TRIGGER_DEADBAND;
                boolean manualFeed = gamepad1.b || gamepad2.right_trigger > TRIGGER_DEADBAND;
                boolean inhibit = gamepad2.left_trigger > TRIGGER_DEADBAND;
                boolean rehome = gamepad1.x && !previousHome && !gamepad1.back
                        && !load && !reverse && !spin && !feed && !manualFeed;
                previousHome = gamepad1.x;
                // Close the gate and re-arm after a switch; a held trigger cannot resume feeding.
                boolean stop = gamepad1.back || rehome || controllerChanged;

                if (rehome) turret.setAllServosToZeroPosition();
                boolean rpmChanged = false, hoodChanged = false;
                if (!stop) {
                    // Coarse Y/A +/-100 and fine D-pad Right/Left +/-10 RPM, one step per press.
                    TARGET_RPM = IntakeShooterTestController.stepRpm(TARGET_RPM,
                            gamepad1.y && !previousRpmUp, gamepad1.a && !previousRpmDown,
                            gamepad1.dpad_right && !previousRpmFineUp, gamepad1.dpad_left && !previousRpmFineDown);
                    double nextRpm = IntakeShooterTestController.stepRpm(TARGET_RPM, false, false, false, false);
                    rpmChanged = nextRpm != selectedRpm;
                    selectedRpm = nextRpm;
                    double hoodDelta = (gamepad1.dpad_up && !previousHoodUp ? PITCH_STEP_DEG : 0)
                            - (gamepad1.dpad_down && !previousHoodDown ? PITCH_STEP_DEG : 0);
                    hoodChanged = hoodDelta != 0;
                    if (hoodChanged) turret.adjustPitchAngleDegrees(hoodDelta);
                }
                previousRpmUp = gamepad1.y;
                previousRpmDown = gamepad1.a;
                previousRpmFineUp = gamepad1.dpad_right;
                previousRpmFineDown = gamepad1.dpad_left;
                previousHoodUp = gamepad1.dpad_up;
                previousHoodDown = gamepad1.dpad_down;

                turret.update();
                if (!stop) turret.updateManualYaw(gamepad1.right_stick_x, dt, false);
                turret.applyYawCommand();
                boolean yawInput = Math.abs(gamepad1.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND;
                // Compare cached measured speed against this loop's selected target.
                double measuredRpm = turret.getShooterCurrentRpm();
                boolean atSpeed = selectedRpm > 0 && Math.abs(measuredRpm - selectedRpm)
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
                boolean feedReady = IntakeShooterTestController.isLooseReady(yawErrorDeg, hoodSettled,
                        selectedRpm, measuredRpm);
                controls.update(now, load, reverse, spin, feed, manualFeed, inhibit, stop,
                        turret.isHomed(), ready, feedReady);
                if (controls.spinFlywheel() && selectedRpm > 0) turret.setShooterRpm(selectedRpm);
                else turret.stopShooter(); // No bench idle: off whenever RT is released.

                gate.setOpen(firing.gateOpen());
                boolean feeding = controls.feedTransfer(gate.isOpenForFeeding());
                if (feeding) minimumFeedRpm = !wasFeeding ? measuredRpm : Math.min(minimumFeedRpm, measuredRpm);
                wasFeeding = feeding;
                transfer.update(now, feeding, controls.pauseLoading(), controls.feedStage1(),
                        controls.forceStage1Loading());
                intake.setIntakePower(controls.intakePower(transfer.isFull()));
                lights.showFaultAlert(now, turret.hasHomingFault(), firing.state() == FiringCoordinator.State.FAULT);
                lights.showIntake(transfer.isFull(), load, reverse, intake.getCommandedPower());
                lights.showShooter(now, firing.state(), controls.manualFeedRequested(), controls.autoShotReady(),
                        turret.getTargetShooterRpm() > 0);
                rumble.update(now, transfer.isFull(), spin, controls.autoShotReady(), gamepad1);

                // Put the exact blocker first so it remains visible above the hardware diagnostics.
                String feedStatus;
                if (stop) feedStatus = "STOP requested";
                else if (!controls.armed()) feedStatus = "Release Back, B, bumpers and triggers to enable";
                else if (inhibit) feedStatus = "BLOCKED (GP2 LT): gate closed, transfer stopped";
                else if (firing.feeding()) feedStatus = (manualFeed ? "MANUAL FEED (GP1 B / GP2 RT): readiness bypassed"
                        : "AUTO FEED: gate open, stage 1 running") + "; release to stop";
                else if (firing.state() == FiringCoordinator.State.GATE_OPENING)
                    feedStatus = String.format("GATE OPENING: stage 1 starts %.2f s after the gate command",
                            IntakeShooterTestController.GATE_OPEN_TO_TRANSFER_SECONDS);
                else if (manualFeed) feedStatus = firing.reason();
                else if (!spin) feedStatus = "Flywheel off: hold RT for selected RPM; RT + LT = auto shot, B / GP2 RT = manual feed";
                else if (selectedRpm <= 0) feedStatus = "Selected RPM is zero: tap Y or D-pad Right to increase";
                else if (!feed) feedStatus = "Spinning: hold LT with RT to shoot when ready";
                else if (!turret.isHomed()) feedStatus = "Home required: " + turret.getHomingStatus();
                else if (firing.state() == FiringCoordinator.State.RECOVERY) feedStatus = firing.reason();
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
                telemetry.addData("Shooting mode", "CONTINUOUS (competition policy) | ready delay %.2f s | gate-to-stage-1 %.2f s",
                        firing.stableSeconds, firing.gateTravelSeconds);
                telemetry.addData("GP1 triggers / bumpers", "RT %.2f | LT %.2f | RB %s | LB %s",
                        gamepad1.right_trigger, gamepad1.left_trigger, load, reverse);
                telemetry.addData("GP2 manual feed / inhibit", "RT %.2f | LT %.2f",
                        gamepad2.right_trigger, gamepad2.left_trigger);
                telemetry.addData("Feed flywheel actual / selected (RPM)", "%.0f / %.0f", measuredRpm, selectedRpm);
                telemetry.addData("Lowest flywheel RPM in last/current feed", minimumFeedRpm);
                telemetry.addData("Last return to RPM tolerance (s)", lastRecoverySeconds);
                if (!Double.isNaN(recoveryStarted)) telemetry.addData("RPM recovery elapsed (s)", now - recoveryStarted);
                telemetry.addData("Feed yaw error / rate (deg, deg/s)", "%.2f / %.2f",
                        yawErrorDeg, turret.getEncoderVelocityDegPerSec());
                telemetry.addData("Motor controls", controls.armed() ? "ENABLED"
                        : "STOPPED: release Back, B, bumpers and triggers to enable");
                telemetry.addData("Selected flywheel (RPM)", "%.0f (hold RT; off when released; Y/A +/-100, D-pad R/L +/-10)",
                        selectedRpm);
                telemetry.addData("Feed checks", "home %s | RPM %s | yaw %s | hood %s | keep feeding %s",
                        turret.isHomed(), atSpeed, yawSettled, hoodSettled, feedReady);
                telemetry.addData("Feed state", "%s | %s", firing.state(), firing.reason());
                telemetry.addData("Feed starts (not ball count)", firing.feedAttempts());
                telemetry.addData("Feed motors", feeding ? "RUNNING" : "STOPPED");
                telemetry.addData("Collection", "%s | intake %+.1f", controls.shootingBusy() ? "PAUSED (shooting)"
                        : inhibit ? "PAUSED (GP2 LT)" : "AUTO", intake.getCommandedPower());
                telemetry.addData("Flywheel regulation", turret.getShooterController());
                lights.addWarningTelemetry(telemetry);
                addControlsTelemetry();
                intake.addTelemetry(telemetry);
                transfer.addTelemetry(telemetry);
                gate.addTelemetry(telemetry);
                turret.addTelemetry(telemetry);
                lights.addTelemetry(telemetry);
                rumble.addTelemetry(telemetry);
                loop.addTelemetry(telemetry);
                if (dashboard != null && now - lastDashboardTime >= DASHBOARD_INTERVAL_MS / 1000.0) {
                    lastDashboardTime = now;
                    TelemetryPacket packet = new TelemetryPacket();
                    // Numeric fields can be selected directly in Dashboard's graph widget.
                    packet.put("Selected RPM", selectedRpm);
                    packet.put("Target RPM", turret.getTargetShooterRpm());
                    packet.put("Measured RPM", measuredRpm);
                    packet.put("Error RPM", turret.getTargetShooterRpm() - measuredRpm);
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
                        turret == null ? null : turret::stop,
                        lights == null ? null : lights::stop,
                        () -> DriverRumble.stop(gamepad1));
            } finally {
                if (dashboard != null) dashboard.setTelemetryTransmissionInterval(previousDashboardIntervalMs);
            }
        }
    }

    /**
     * Dashboard's static setup needs the robot's native RobotCore library. On a desktop JVM (SimLoop) it fails
     * after already starting a telemetry thread, so it is not touched off Android (ART reports "Dalvik").
     */
    private static FtcDashboard dashboardOrNull() {
        if (!"Dalvik".equals(System.getProperty("java.vm.name"))) return null;
        try {
            return FtcDashboard.getInstance();
        } catch (LinkageError e) {
            return null;
        }
    }

    private static double bounded(double value, double fallback, double min, double max) {
        return Double.isFinite(value) ? Math.max(min, Math.min(max, value)) : fallback;
    }

    private void addControlsTelemetry() {
        telemetry.addLine("Dashboard: 192.168.43.1:8080/dash | Config: IntakeShooterTestOpMode");
        telemetry.addLine("Live P/I/D/F_SCALE/TARGET_RPM and FUSION_KS/KV/KP; applied gains shown below; GP2 X resets all gains");
        telemetry.addLine(String.format("Shooting = competition TeleOp: CONTINUOUS feed; bench-only: stage 1 starts %.2f s after the gate opens",
                IntakeShooterTestController.GATE_OPEN_TO_TRANSFER_SECONDS));
        telemetry.addLine("Automatic intake/index after arming: LOW forward, 0.5 s HIGH stop; paused while shooting");
        telemetry.addLine("GP1/GP2 RB forces intake + stage 1 forward / LB reverses intake / Both stop");
        telemetry.addLine("Flywheel off unless RT held (no idle) | Hold RT = Selected RPM | RT + LT = Auto Shot once ready (0.15 s)");
        telemetry.addLine("GP1 B or GP2 RT = Manual Feed (readiness bypassed) | GP2 LT = Inhibit (gate closed, transfer stopped)");
        telemetry.addLine("GP2 PID tuning (PIDF only): Dpad U/D = P +/-1 | R/L = F scale +/-0.05 | Y/A = I +/-0.05 | X = Reset All Gains");
        telemetry.addLine("GP2 B = Toggle Fusion (default) / PIDF; release GP1/GP2 motor controls to re-arm");
        telemetry.addLine("Y/A = RPM +/-100 | Dpad R/L = RPM +/-10 | Dpad Up = Lower Hood, Down = Raise Hood (0.5 deg) | RS-X = Turret");
        telemetry.addLine("Back = Stop Motors + Close Gate (release controls to resume) | X = Rehome with motor controls released");
    }
}
