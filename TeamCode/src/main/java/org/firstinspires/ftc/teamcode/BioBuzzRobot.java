package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.horizon36596.zenith.ivy.AutoFromFile;
import org.horizon36596.zenith.ivy.ZenithRobot;
import org.horizon36596.zenith.AutoFile;
import org.horizon36596.zenith.ivy.NamedCommands;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Shared Pedro + Ivy lifecycle. Commands use cached sensors and a single output coordinator. */
public final class BioBuzzRobot implements ZenithRobot {
    private final TurretSubsystem.Alliance selectedAlliance;
    private final java.util.function.LongSupplier clock;
    final AutonomousProfile profile;
    final AutonomousOutputCoordinator outputs = new AutonomousOutputCoordinator();
    TurretSubsystem.Alliance alliance;
    private RobotLoop loop;
    PedroDrive drive;
    IntakeSubsystem intake;
    TransferSubsystem transfer;
    GateSubsystem gate;
    DepollinatorSubsystem depollinator;
    TurretSubsystem turret;
    ShotCoordinator shots;
    FiringCoordinator firing;
    PollenVision vision;
    PollenVision.Snapshot pollenSnapshot;
    boolean running;
    private boolean hasStarted;
    private long started;
    String collectionStatus = "Idle";
    private String activeStep = "INIT", outcome = "NONE", outcomeReason = "No bounded action completed", outcomeStep = "INIT";
    private double outcomeAt;
    private boolean terminalOutcome;
    private final Map<String, String> conditionWaits = new HashMap<>();
    public static final double MATCH_TIME_LOW_SECONDS = 5;

    public BioBuzzRobot() { this(null, AutonomousProfile.FULL_ROBOT); }
    public BioBuzzRobot(TurretSubsystem.Alliance alliance) { this(alliance, AutonomousProfile.FULL_ROBOT); }
    private BioBuzzRobot(TurretSubsystem.Alliance alliance, AutonomousProfile profile) {
        this(alliance, profile, System::nanoTime);
    }
    BioBuzzRobot(TurretSubsystem.Alliance alliance, AutonomousProfile profile, java.util.function.LongSupplier clock) {
        selectedAlliance = alliance; this.profile = profile; this.clock = clock;
    }
    public static BioBuzzRobot pollenTest() { return new BioBuzzRobot(null, AutonomousProfile.POLLEN_TEST); }

    @Override public void init(AutoFromFile opMode) {
        AutoPoseHandoff.clear(); // An older run's pose must never reach the next TeleOp.
        observeSteps(opMode.spec().steps);
        alliance = selectedAlliance != null ? selectedAlliance : TurretSubsystem.Alliance.valueOf(opMode.spec().alliance);
        try {
            loop = new RobotLoop(opMode.hardwareMap);
            drive = new PedroDrive(opMode.hardwareMap, profile != AutonomousProfile.POLLEN_TEST);
            intake = new IntakeSubsystem(opMode.hardwareMap);
            transfer = new TransferSubsystem(opMode.hardwareMap);
            gate = new GateSubsystem(opMode.hardwareMap, this::now);
            if (profile == AutonomousProfile.FULL_ROBOT) {
                depollinator = new DepollinatorSubsystem(opMode.hardwareMap, this::now);
                turret = new TurretSubsystem(opMode.hardwareMap, true, this::now);
                shots = new ShotCoordinator(turret, TeleOpFieldReference.target(alliance, TurretSubsystem.HiveSide.AUDIENCE),
                        ShotCoordinator.AimMode.AUTONOMOUS_ODOMETRY);
            }
            firing = shots == null ? ShootingConfiguration.firing() : shots.firing();
            // Camera is optional for planned paths; camera-dependent factories fail during INIT.
            try {
                if (opMode.hardwareMap.tryGet(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                        PollenVision.HW_WEBCAM) != null) vision = new PollenVision(opMode.hardwareMap);
                else opMode.telemetry.addLine("Pollen camera not configured; camera actions unavailable.");
            }
            catch (RuntimeException e) { opMode.telemetry.addData("Pollen camera unavailable", e.getMessage()); }
        } catch (RuntimeException failure) {
            try { terminalStop(); } catch (RuntimeException cleanup) { failure.addSuppressed(cleanup); }
            throw failure;
        }
    }
    @Override public void registerCommands(AutoFromFile opMode) { new AutonomousCommands(this).register(); }
    double now() { return nanoTime() * 1e-9; }
    @Override public long nanoTime() { return clock.getAsLong(); }
    double remainingSeconds() { return hasStarted ? Math.max(0, PedroConfiguration.MAX_AUTONOMOUS_SECONDS - (nanoTime() - started) * 1e-9) : PedroConfiguration.MAX_AUTONOMOUS_SECONDS; }
    void outcome(String result, String reason) {
        if (terminalOutcome) return;
        outcome = result; outcomeReason = reason; outcomeAt = now(); outcomeStep = activeStep;
    }
    /** Shutdown end hooks must not replace the fault/deadline that caused cancellation. */
    void terminalOutcome(String result, String reason) {
        outcome(result, reason); terminalOutcome = true;
    }
    void observeSteps(List<AutoFile.Step> steps) {
        for (AutoFile.Step step : steps) {
            if (step instanceof AutoFile.WaitStep && ((AutoFile.WaitStep) step).until != null)
                conditionWaits.put(step.id, ((AutoFile.WaitStep) step).until);
            if (step instanceof AutoFile.SequenceStep) observeSteps(((AutoFile.SequenceStep) step).steps);
            if (step instanceof AutoFile.ParallelStep) observeSteps(((AutoFile.ParallelStep) step).steps);
            if (step instanceof AutoFile.BranchStep) {
                observeSteps(((AutoFile.BranchStep) step).then);
                observeSteps(((AutoFile.BranchStep) step).otherwise);
            }
        }
    }
    boolean commandFailed() { return !outcome.equals("NONE") && !outcome.equals("SUCCESS"); }
    String lastOutcome() { return outcome; }
    @Override public Command step(String id, Command command) {
        return new CommandBuilder().requiring(command.requirements()).setPriority(command.priority())
                .setInterruptedBehavior(command.interruptedBehavior()).setBlockedBehavior(command.blockedBehavior())
                .setConflictBehavior(command.conflictBehavior())
                .setStart(() -> { activeStep = id; command.start(); })
                .setExecute(() -> { activeStep = id; command.execute(); }).setDone(command::done)
                .setEnd(reason -> {
                    activeStep = id; command.end(reason);
                    String condition = conditionWaits.get(id);
                    if (condition != null) {
                        boolean satisfied = NamedCommands.condition(condition).getAsBoolean();
                        outcome(reason != EndCondition.NATURALLY ? "CANCELLED" : satisfied ? "SUCCESS" : "TIMEOUT",
                                "Condition wait: " + condition);
                    }
                });
    }
    @Override public Follower follower() { return drive.follower(); }
    @Override public Object drive() { return drive; }
    @Override public String runningAlliance() { return alliance.name(); }
    @Override public Pose mirror(Pose p) { return new Pose(-p.x(), -p.y(), p.heading() + Math.PI); }
    @Override public Path atSpeed(Path path, double fraction) { return drive.atSpeed(path, fraction); }
    @Override public void setStartPose(Pose pose) { drive.setStartPose(pose); }
    private void readSensors() {
        loop.begin(); drive.readSensors();
        transfer.readSensors(now());
        if (turret != null) turret.update(now());
        pollenSnapshot = vision == null ? null : vision.snapshot(nanoTime());
    }
    @Override public void initLoop(AutoFromFile opMode) {
        readSensors();
        if (turret != null) turret.applyYawCommand(now());
        opMode.telemetry.addLine("Keep robot stationary during Pinpoint calibration.");
        opMode.telemetry.addData("Autonomous hardware profile", profile);
        drive.addTelemetry(opMode.telemetry);
        if (turret != null) turret.addTelemetry(opMode.telemetry);
        if (vision != null) vision.addTelemetry(opMode.telemetry);
        loop.addTelemetry(opMode.telemetry);
    }
    @Override public void onStart(AutoFromFile opMode) {
        if (!PedroConfiguration.TUNING_VERIFIED) throw new IllegalStateException("Measure and verify PedroConfiguration before running autonomous; use Test: Pedro Drive");
        if (!drive.ready(now())) throw new IllegalStateException("Pinpoint has no READY, fresh autonomous start pose");
        started = nanoTime(); hasStarted = running = true;
    }
    @Override public void periodic(AutoFromFile opMode) {
        readSensors();
        if (!drive.ready(now())) {
            AutoPoseHandoff.clear(); // A pose from before a localization fault is not trustworthy for TeleOp.
            terminalOutcome("FAULT", "Pinpoint localization unavailable or stale");
            IllegalStateException failure = new IllegalStateException(outcomeReason);
            try { stopOutputs(); } catch (RuntimeException cleanup) { failure.addSuppressed(cleanup); }
            throw failure;
        }
        // Field-referenced routes only (a cleared-area local start is ignored); TeleOp may start from it.
        AutoPoseHandoff.save(alliance, drive.sample());
        if (remainingSeconds() <= 0) { terminalOutcome("TIMEOUT", "Global autonomous deadline"); finish(opMode); return; }
        drive.update();
    }
    @Override public void afterLoop(AutoFromFile opMode) {
        if (running && !Scheduler.isScheduled(opMode.routine())) finish(opMode);
        if (running) outputs.apply(now(), drive.ready(now()) ? drive.sample() : null, turret, shots, firing, gate, transfer, intake);
        else if (turret != null) turret.applyYawCommand(now());
        drive.addTelemetry(opMode.telemetry);
        opMode.telemetry.addData("Routine / profile", "%s | %s", opMode.spec().name, profile);
        opMode.telemetry.addData("Autonomous step / remaining (s)", "%s | %.2f", activeStep, remainingSeconds());
        opMode.telemetry.addData("Last bounded action", "%s | %s | %s | %.3f s", outcomeStep, outcome, outcomeReason, outcomeAt);
        opMode.telemetry.addData("Output modes yaw / hood / flywheel", "%s / %s / %s", outputs.yawMode(), outputs.hoodMode(), outputs.flywheelMode());
        opMode.telemetry.addData("Feed inhibited", outputs.inhibited());
        if (turret != null) opMode.telemetry.addData("Targets yaw / hood / flywheel (deg, deg down, RPM)",
                "%.1f / %.1f / %.0f", turret.getTargetYawDegrees(), turret.getTargetPitchDegrees(), turret.getTargetShooterRpm());
        opMode.telemetry.addData("Pollen navigation test", collectionStatus);
        transfer.addTelemetry(opMode.telemetry, now()); gate.addTelemetry(opMode.telemetry);
        if (depollinator != null) depollinator.addTelemetry(opMode.telemetry);
        if (shots != null) shots.addTelemetry(opMode.telemetry, now());
        if (vision != null) vision.addTelemetry(opMode.telemetry);
        loop.addTelemetry(opMode.telemetry); opMode.telemetry.update();
    }
    private void finish(AutoFromFile opMode) {
        running = false;
        RobotShutdown.stopAll(() -> Scheduler.cancel(opMode.routine()), this::stopOutputs, opMode::requestOpModeStop);
    }
    /** In-routine stop retains references and positional posture; it is resumable. */
    void stopOutputs() {
        outputs.stopAll();
        RobotShutdown.stopAll(firing == null ? null : () -> firing.stop(now()),
                drive == null ? null : drive::stop, intake == null ? null : intake::stop,
                transfer == null ? null : transfer::pause, gate == null ? null : gate::close,
                turret == null ? null : turret::holdYaw, turret == null ? null : turret::stopShooter);
    }
    private void terminalStop() {
        running = false;
        RobotShutdown.stopAll(this::stopOutputs, turret == null ? null : turret::stop, vision == null ? null : vision::close);
    }
    @Override public void onStop(AutoFromFile opMode) {
        running = false;
        RobotShutdown.stopAll(opMode.routine() == null ? null : () -> Scheduler.cancel(opMode.routine()), this::terminalStop);
    }
}
