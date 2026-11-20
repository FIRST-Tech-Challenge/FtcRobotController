package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/**
 * Every robot mechanism for OpModes exported by the BioBuzz Path Planner (tools/path-planner, Pedro 3
 * target). Plain class: no waitForStart(), loops or sleep(). The exported OpMode calls, once per loop:
 * {@link #readSensors(double)} after the Pinpoint read, the current planner step, then {@link #apply(double)};
 * during INIT {@link #initLoop(double)} replaces apply().
 *
 * <p>Mechanisms are driven the way the competition and fallback autos drive them: IntakeSubsystem (TeleOp's
 * direction, forward collection stops on a full queue), TransferSubsystem (full sensor + stage 1 run-on),
 * GateSubsystem with the shared FiringCoordinator continuous feed (CompetitionFeedController timing), and
 * TurretSubsystem (homing, rear dead zone, encoder-verified yaw, shooter PIDF), aimed with the competition
 * pivot geometry and aim lead; hood / RPM come from ShootingConfiguration.PROFILE.
 *
 * <p>Like {@link BioBuzzRobot}, with tag relocalization on the Limelight refines the authored start pose in
 * INIT ({@link AutoStartRelocalization}) and ShotCoordinator's AUTONOMOUS_RELOCALIZING pose corrects the
 * aiming pose during the route; Pedro paths keep the Pinpoint pose. The depollinator, linear slide (with
 * its fixed shot blocker), status lights and pollen camera are optional: an unconfigured device is reported in telemetry and
 * its planner steps become no-ops, so a missing mechanism never stops the auto.
 */
public final class PlannerRobot {
    static final double METERS_PER_INCH = FieldGeometry.METERS_PER_INCH;

    /** Intake power for {@link #setIntake(int)} mode 1 / -1 (the planner's T.INTAKE_POWER). */
    public double intakePowerScale = 1.0;
    /** Continuous feed time for {@link #startShoot()} (the planner's T.FEED_SECONDS). */
    public double feedSeconds = FallbackAutoController.FEED_SECONDS;

    public final PedroDrive drive;
    public final IntakeSubsystem intake;
    public final TransferSubsystem transfer;
    public final GateSubsystem gate;
    public final TurretSubsystem turret;
    /** Optional mechanisms: null when not configured on this robot. */
    public final DepollinatorSubsystem depollinator;
    public final LinearSlideSubsystem slide;
    public final RobotStatusLights lights;
    public final PollenVision vision;
    /** Tag relocalization (start-pose refinement + aim-pose correction); null when off or no Limelight. */
    public final ShotCoordinator shots;

    final CompetitionFeedController feedControl = new CompetitionFeedController(); // continuous feed
    final FiringCoordinator firing = CompetitionFeedController.firing();
    private AutoStartRelocalization relocalization = new AutoStartRelocalization();
    private TurretSubsystem.Alliance alliance;
    private final List<String> missing = new ArrayList<>();

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

    /** Last shot / mechanism note shown in telemetry; steps may set it. */
    public String shotNote = "";
    int homeRetries = 0;                 // turret homes restarted after a FAULT
    private boolean started = false;

    // ---- pollen collection (same navigator + follower as the Zenith collectPollen command) ----
    private PollenVision.Snapshot pollenSnapshot;
    private final PollenNavigator navigator = new PollenNavigator(PollenVision.CONFIG);
    private final PollenPathFollower navigation;
    private boolean collecting = false;
    private String collectionStatus = "Idle";

    /**
     * @param tagRelocalization use the Limelight like the competition autos (BioBuzzRobot.TAG_RELOCALIZATION_ENABLED)
     * @param pollenCamera      start the C270 pollen camera (only when the plan has a Collect pollen step)
     */
    public PlannerRobot(HardwareMap hw, PedroDrive drive, TurretSubsystem.Alliance alliance,
                        boolean tagRelocalization, boolean pollenCamera) {
        this.drive = drive;
        this.alliance = alliance;
        intake = new IntakeSubsystem(hw);        // never resets intake_motor's encoder (turret encoder channel)
        transfer = new TransferSubsystem(hw);
        gate = new GateSubsystem(hw, CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS);
        // Starts non-blocking turret homing and puts the hood at home.
        turret = new TurretSubsystem(hw, tagRelocalization);
        turret.setEncoderCorrectionEnabled(true);
        shots = tagRelocalization && turret.hasLimelight()
                ? new ShotCoordinator(turret, TeleOpFieldReference.target(alliance, TurretSubsystem.HiveSide.AUDIENCE),
                        ShotCoordinator.AimMode.AUTONOMOUS_RELOCALIZING)
                : null;
        if (tagRelocalization && shots == null) missing.add("limelight (tag relocalization off)");
        depollinator = optional("depollinator", () -> new DepollinatorSubsystem(hw));  // homes at INIT
        slide = optional("linear slide", () -> new LinearSlideSubsystem(hw));         // must be fully down at INIT
        lights = new RobotStatusLights(hw);                                           // reports its own missing lights
        PollenVision camera = null;
        if (pollenCamera) {
            camera = hw.tryGet(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class,
                    PollenVision.HW_WEBCAM) == null ? null : optional("pollen camera", () -> new PollenVision(hw));
            if (camera == null && !missing.contains("pollen camera")) missing.add("pollen camera");
        }
        vision = camera;
        navigation = new PollenPathFollower(drive);
    }

    private <T> T optional(String name, Supplier<T> factory) {
        try { return factory.get(); }
        catch (RuntimeException e) { missing.add(name); return null; }
    }

    private static double now() { return System.nanoTime() * 1e-9; }
    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    /** INIT alliance select changed: target the new alliance and restart the start-pose refinement. */
    public void setAlliance(TurretSubsystem.Alliance next) {
        if (next == alliance) return;
        alliance = next;
        if (shots != null) {
            shots.cancelFieldRehomeRefinement("alliance changed");
            shots.select(TeleOpFieldReference.target(next, TurretSubsystem.HiveSide.AUDIENCE));
            shots.invalidateReference(now());
        }
        relocalization = new AutoStartRelocalization();
    }

    // =====================================================================================
    //  COMMANDS - call from planner steps, path callbacks and loop code
    // =====================================================================================
    /** Intake in (1), out (-1) or off (0) at intakePowerScale; forward collection stops once the transfer is full. */
    public void setIntake(int mode) { intakePower = mode * intakePowerScale; }
    public void spin(boolean on) { spinning = on; }
    public void setRpmAuto() { rpmAuto = true; }
    /** Flywheel output RPM, the same units as TeleOp and the shot table. */
    public void setRpmManual(double rpm) { rpmAuto = false; rpmManual = rpm; }
    public void setHoodAuto() { hoodAuto = true; }
    /** Hood travel down from home in degrees (0 = home / highest launch, 30 = lowest). */
    public void setHoodManual(double travelDeg) {
        hoodAuto = false;
        hoodManualDeg = clamp(travelDeg, TurretSubsystem.PITCH_MIN_DEG, TurretSubsystem.PITCH_MAX_DEG);
    }
    public void aimAt(Pose target) { aimTarget = target; }
    public void setTracking(boolean on) { tracking = on; if (on) fixedHeadingDeg = Double.NaN; }
    /** Fixed turret heading in the Driver Hub frame: front 0, right (home) 90, back 180, left 270 deg. */
    public void setTurretHeading(double headingDeg) { tracking = false; fixedHeadingDeg = headingDeg; }
    /** Hold the gate open (true) or let the feed control it again (false). */
    public void gateOpen(boolean open) { gateHeldOpen = open; }

    /**
     * Shot: the fallback auto's continuous feed (gate, then both transfer stages) for feedSeconds. A
     * continuous feed cannot count launched balls (FiringCoordinator), so it feeds every loaded ball.
     */
    public void startShoot() { feedUntil = now() + feedSeconds; shotNote = ""; }
    public boolean isShooting() { return now() < feedUntil; }
    public boolean turretHomed() { return turret.isHomed(); }
    public boolean turretFault() { return turret.hasHomingFault(); }
    public boolean transferFull() { return transfer.isFull(); }

    /** Restart turret homing (turret bumped, or before a precise shot). Feeding waits for the new home. */
    public void rehomeTurret() {
        feedUntil = Double.NEGATIVE_INFINITY;
        turret.homeTurretAndZeroEncoder(now());
        if (shots != null) shots.invalidateReference(now());
    }

    /** Encoder-verified turret on its target (homed, fresh pose while tracking). */
    public boolean turretReady() {
        return turret.isHomed() && (!tracking || poseFresh)
                && Math.abs(turret.getTargetYawDegrees() - turret.getEncoderAngleDegrees())
                <= TurretSubsystem.SHOT_READY_YAW_TOLERANCE_DEG;
    }

    /** Same readiness as the fallback auto: flywheel at speed, hood settled and turret on target. */
    public boolean ready() {
        return spinning && rpmTarget > 0 && turret.isShooterAtSpeed() && turret.isHoodTracking() && turretReady();
    }

    // ---- depollinator ----
    public void deployDepollinator() { if (depollinator != null) depollinator.deploy(); else note("depollinator"); }
    public void homeDepollinator() { if (depollinator != null) depollinator.home(); else note("depollinator"); }
    /** Settle estimate (no position feedback); true when the depollinator is not configured. */
    public boolean depollinatorSettled() { return depollinator == null || depollinator.isSettledEstimated(); }

    // ---- linear slide (the shot blocker is fixed to it) ----
    public void raiseSlide() { if (slide != null) slide.raise(); else note("linear slide"); }
    public void lowerSlide() { if (slide != null) slide.lower(); else note("linear slide"); }
    /** Slide reached its commanded end (UP / DOWN) or faulted; true when the slide is not configured. */
    public boolean slideSettled() {
        if (slide == null) return true;
        LinearSlideController.State s = slide.getState();
        return s == LinearSlideController.State.FAULT
                || s == (slide.isUpCommanded() ? LinearSlideController.State.UP : LinearSlideController.State.DOWN);
    }
    public boolean slideFault() { return slide != null && slide.getState() == LinearSlideController.State.FAULT; }

    // ---- pollen collection: PollenNavigator drives the follower and intake until it finishes ----
    /** Start a camera-guided pollen collection; the planner step must not command the follower meanwhile. */
    public void startCollect() {
        if (vision == null) { note("pollen camera"); collectionStatus = "SKIPPED: pollen camera not configured"; return; }
        feedUntil = Double.NEGATIVE_INFINITY;
        navigation.stop();
        MotionSample p = drive.sample();
        navigator.start(System.nanoTime(), p.x * 1000, p.y * 1000);
        collecting = true;
    }
    /** True once the collection finished (complete, timed out, fault) or cannot run. */
    public boolean collectDone() { return !collecting || navigator.isFinished(); }
    public void stopCollect() {
        if (!collecting) return;
        collecting = false;
        navigator.stop(System.nanoTime());
        navigation.stop();
        intakePower = 0;
    }
    public boolean pollenVisible() {
        return pollenSnapshot != null && pollenSnapshot.autonomousReady && pollenSnapshot.target != null;
    }

    // ---- tag relocalization ----
    /** INIT start-pose refinement finished and was written into Pinpoint. */
    public boolean relocalized() { return relocalization.state() == AutoStartRelocalization.State.REFINED; }

    private void note(String device) { shotNote = "SKIPPED: " + device + " not configured"; }

    /** Mid-auto stop: mechanisms off, the turret holds its last target and keeps its home; slide holds. */
    public void stopAll() {
        stopCollect();
        intakePower = 0;
        spinning = false;
        tracking = false;
        fixedHeadingDeg = Double.NaN;
        gateHeldOpen = false;
        feedUntil = Double.NEGATIVE_INFINITY;
        double t = now();
        RobotShutdown.stopAll(() -> firing.stop(t), intake::stop, transfer::pause, gate::close, turret::stopShooter);
    }

    /** End of OpMode: every output stops even if one device throws. */
    public void shutdown() {
        RobotShutdown.stopAll(this::stopAll, transfer::stop, gate::close, turret::stop,
                slide == null ? null : slide::stop, lights::stop, vision == null ? null : vision::close);
    }

    // =====================================================================================
    //  LOOP
    // =====================================================================================
    /**
     * Once per loop, after drive.readSensors() and before the step: transfer full sensor and stage 1 speed,
     * turret encoder, homing and flywheel velocity, the Limelight aim-pose correction and the pollen camera
     * frame. A home that faults (turret bumped or still moving in its settle window) restarts at once;
     * otherwise the turret would stay unhomed and never aim for the rest of the auto.
     */
    public void readSensors(double now) {
        transfer.readSensors(now);
        turret.update(now);
        if (turret.hasHomingFault()) { homeRetries++; turret.homeTurretAndZeroEncoder(now); }
        if (shots != null) shots.updatePose(now, drive.ready(now) ? drive.sample() : null);
        pollenSnapshot = vision == null ? null : vision.snapshot(System.nanoTime());
    }

    /**
     * INIT loop, robot stationary: turret homing travel and, with tag relocalization, the start-pose refinement
     * (BioBuzzRobot.relocalizeStart). Nothing spins or feeds before START; the slide stays unpowered.
     */
    public void initLoop(double now) {
        if (shots != null) {
            boolean poseReady = drive.ready(now);
            MotionSample seeded = poseReady ? drive.sample() : null;
            switch (relocalization.update(turret.isHomed(), poseReady, shots.fieldRehomePending(), shots.fieldRehomeStatus())) {
                case REQUEST_REFINEMENT:
                    shots.requestFieldRehomeRefinement(now, AutoStartRelocalization.landmarkFor(seeded.y));
                    break;
                case APPLY_REFINED_POSE:
                    MotionSample refined = shots.pose();
                    drive.setFieldPose(BioBuzzRobot.pedroPose(refined));
                    relocalization.applied(Math.hypot(refined.x - seeded.x, refined.y - seeded.y) / METERS_PER_INCH);
                    break;
                default:
                    break;
            }
        }
        turret.applyYawCommand(now); // Homing travel and the refinement's look yaw.
        if (slide != null) slide.trackWithoutDriving();
        showFaultAlert(now);
        showReference(now);
    }

    /** At START: a refinement still collecting frames ends; the start pose stays. */
    public void onStart() {
        started = true;
        if (shots != null) shots.cancelFieldRehomeRefinement("START before the refinement finished");
    }

    /**
     * Once per loop, after the step. Aim uses the competition auto-aim rules: pivot offset from
     * ShootingConfiguration.MOUNT, the TurretControl aim lead, and TurretSubsystem's reachable-yaw rule for the
     * rear dead zone. With tag relocalization the aim uses the camera-corrected pose.
     */
    public void apply(double now) {
        MotionSample raw = drive.sample();
        MotionSample corrected = shots == null ? null : shots.pose();
        MotionSample pose = corrected != null && corrected.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS) ? corrected : raw;
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

        // ---- pollen collection owns the follower and intake power while it runs ----
        if (collecting) {
            long nanos = System.nanoTime();
            MotionSample p = drive.sample();
            navigator.update(nanos, pollenSnapshot == null ? null : pollenSnapshot.target,
                    p.x * 1000, p.y * 1000, Math.toDegrees(p.heading),
                    drive.ready(now) && pollenSnapshot != null && pollenSnapshot.autonomousReady, transfer.isFull());
            intakePower = navigator.getIntakePower();
            collectionStatus = navigator.getState() + ": " + navigator.getStatus();
            navigation.update(navigator, nanos);
        }

        // ---- feed: the fallback auto's manual-feed path (TeleOp GP1 A): gate, then the transfer ----
        boolean feedRequested = now < feedUntil;
        feedControl.updateSelection(now, firing);   // continuous feed only (CompetitionFeedController)
        firing.update(now, false, feedRequested, false, false, false, false, false, "Planner autonomous feed");
        boolean feedStage1 = feedControl.feedStage1(now, firing);
        gate.setOpen(firing.gateOpen() || gateHeldOpen);
        boolean shootingBusy = feedRequested || firing.gateOpen()
                || firing.state() == FiringCoordinator.State.RECOVERY;
        // Stage 1 loads (full sensor + run-on) while intaking or spinning up for a shot.
        boolean loading = intakePower > 0 || spinning;
        transfer.updateCached(now, feedControl.feedTransfers(firing, gate.isOpenForFeeding()),
                shootingBusy || !loading, feedStage1, false);
        // Forward collection stops on a confirmed full queue (competition auto rule); reverse still allowed.
        intake.setIntakePower(transfer.isFull() && intakePower > 0
                ? IntakeQueueController.FULL_QUEUE_INTAKE_POWER : intakePower);

        if (slide != null) { if (started) slide.update(); else slide.trackWithoutDriving(); }

        // ---- status lights: TeleOp's colors; a new fault blinks all three red first ----
        showFaultAlert(now);
        lights.showIntake(transfer.isFull(), false, false, intake.getCommandedPower());
        lights.showShooter(now, firing.state(), feedRequested, ready(), turret.getTargetShooterRpm() > 0);
        showReference(now);
    }

    private void showFaultAlert(double now) {
        lights.showFaultAlert(now, turret.hasHomingFault(), slideFault(),
                firing.state() == FiringCoordinator.State.FAULT);
    }

    private void showReference(double now) {
        MotionSample p = shots != null && shots.pose() != null ? shots.pose() : drive.sample();
        lights.showReference(!turret.hasHomingFault(), turret.isHomed(),
                p != null && p.fieldReferenced && p.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS));
    }

    public void addTelemetry(Telemetry t) {
        turret.addDriverTelemetry(t);
        transfer.addDriverTelemetry(t);
        if (homeRetries > 0) t.addData("Turret home retries", homeRetries);
        t.addData("Turret target heading (deg CW)", "%.1f | %s", turret.getTargetRobotHeadingDegrees(),
                !turret.isHomed() ? "NOT HOMED" : turretReady() ? "ON TARGET" : tracking && !poseFresh ? "POSE STALE" : "MOVING");
        t.addData("Aim range (in)", Double.isFinite(dist)
                ? String.format(Locale.US, "%.1f", dist) : "no aim point (preload settings)");
        t.addData("Hood travel (deg) / flywheel target (RPM)", "%.1f / %.0f", hoodTargetDeg, rpmTarget);
        t.addData("Shot", "ready %s | feed %s%s", ready(), firing.state(), shotNote.isEmpty() ? "" : " | " + shotNote);
        if (shots != null) t.addData("Tag relocalization", "%s | %s", relocalization.status(), shots.visionDecision());
        if (shots != null && relocalization.checkPlacement()) t.addData("CHECK PLACEMENT",
                "hive tags disagree with the authored start pose by more than %.0f in",
                FieldRehomeRefinement.MAX_CORRECTION_METERS * PedroConfiguration.INCHES_PER_METER);
        if (depollinator != null) depollinator.addTelemetry(t);
        if (slide != null) slide.addDriverTelemetry(t);
        if (vision != null) t.addData("Pollen collection", collectionStatus);
        if (!missing.isEmpty()) t.addData("Not configured (steps skipped)", String.join(", ", missing));
        lights.addWarningTelemetry(t);
    }
}
