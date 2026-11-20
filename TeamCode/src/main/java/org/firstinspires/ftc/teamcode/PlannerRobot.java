package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Every robot function for OpModes exported by the BioBuzz Path Planner (tools/path-planner, Pedro 3 target). Plain
 * class: no waitForStart(), loops or sleep(). The exported OpMode calls, once per loop: {@link #readSensors(double)}
 * after the Pinpoint read, the current planner step, the drive update, then {@link #apply(double)}; during INIT
 * {@link #initLoop(double)} replaces apply().
 *
 * <p>Shooting, feeding, the transfer and the intake run through {@link CompetitionShotPipeline}, the same code as
 * competition TeleOp: {@link ShotCoordinator} aims the turret and sets hood / RPM from
 * {@code ShootingConfiguration.PROFILE} (TeleOp RT), with Limelight tag pose corrections when tag relocalization is
 * on; the shared {@link FiringCoordinator} waits for TeleOp's readiness (turret on target, hood settled, flywheel at
 * speed, stable for {@code READY_STABLE_SECONDS}) and feeds continuously (RT + LT); a forced feed bypasses
 * readiness (GP1 A); {@link IntakeQueueController}'s full-queue rule stops collection. The flywheel uses TeleOp's
 * controller ({@link CompetitionShotPipeline#FLYWHEEL_CONTROLLER}). {@link PlannerVolley} sequences one volley:
 * aim, ready, feed, finish / timeout.
 *
 * <p>Like {@link BioBuzzRobot}, with tag relocalization on the Limelight refines the authored start pose in INIT
 * ({@link AutoStartRelocalization}) and corrects the aiming pose during the route; a Relocalize step repeats the
 * refinement mid-route with the robot stopped. The depollinator, linear slide (with its fixed shot blocker),
 * status lights and pollen camera are optional: an unconfigured device is reported in telemetry and its planner
 * steps become no-ops, so a missing mechanism never stops the auto.
 */
public final class PlannerRobot {
    static final double METERS_PER_INCH = FieldGeometry.METERS_PER_INCH;
    /** An aim point within this distance of a hive opening (in) aims with TeleOp's exact cell target. */
    public static final double CELL_MATCH_INCHES = 1.0;

    /** OFF: collection paused. IN: TeleOp's automatic collection (stops on a full queue). FORCE: TeleOp RB. REVERSE: LB. */
    public enum IntakeMode { OFF, IN, FORCE, REVERSE }
    /** HOLD: keep the last yaw target. TRACK: auto aim yaw. FIXED: a fixed Driver Hub heading. */
    public enum YawMode { HOLD, TRACK, FIXED }
    /** OFF, TeleOp's idle RPM, a manual RPM, or the shot table (TeleOp RT). */
    public enum Flywheel { OFF, IDLE, MANUAL, AUTO }
    /** After a volley / shot: keep aiming and spinning, drop to TeleOp's idle RPM, or spin down. */
    public enum After { KEEP, IDLE, OFF }

    /** Intake power scale (the planner's T.INTAKE_POWER). */
    public double intakePowerScale = 1.0;
    /** Default feed time for legacy {@link #startShoot()} (the planner's T.FEED_SECONDS). */
    public double feedSeconds = FallbackAutoController.FEED_SECONDS;
    /** Default ready timeout for legacy {@link #startShoot()} (the planner's T.READY_TIMEOUT_S). */
    public double readyTimeoutSeconds = FallbackAutoController.SPIN_UP_TIMEOUT_SECONDS;

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
    /** TeleOp's aiming / readiness / firing owner. Camera-corrected when tag relocalization is on. */
    public final ShotCoordinator shots;

    final FiringCoordinator firing;
    final CompetitionShotPipeline pipeline = new CompetitionShotPipeline();
    final CompetitionShotPipeline.Request request = new CompetitionShotPipeline.Request();
    final AutoAimAdjustments trims = new AutoAimAdjustments();
    final PlannerVolley volley = new PlannerVolley();
    private final boolean tagRelocalization;
    private final DoubleSupplier clock;
    private AutoStartRelocalization relocalization = new AutoStartRelocalization();
    private TurretSubsystem.Alliance alliance;
    private final List<String> missing = new ArrayList<>();

    IntakeMode intakeMode = IntakeMode.OFF;
    double intakeOffAt = Double.POSITIVE_INFINITY;   // timed intake ends here (s)
    boolean intakeOffWhenFull = false;
    YawMode yawMode = YawMode.HOLD;
    double fixedHeadingDeg = Double.NaN;             // front 0, right 90, back 180, left 270
    boolean hoodAuto = false;
    double hoodManualDeg = TurretSubsystem.PITCH_HOME_DEG; // hood travel down from home, 0..30 deg
    Flywheel flywheel = Flywheel.OFF;
    boolean rpmAuto = true;                          // legacy spin(): AUTO or MANUAL when spinning
    double rpmManual = CompetitionShotPipeline.FLYWHEEL_IDLE_RPM;
    boolean hasAimPoint = false;                     // an aim point or nearest-cell target was chosen
    After after = After.KEEP;
    boolean gateHeldOpen = false;
    boolean holdLoading = false;
    boolean poseFresh = false;

    private RobotStatusLights.Color lightsColor = RobotStatusLights.Color.OFF;
    private double lightsUntil = Double.NEGATIVE_INFINITY;

    /** Last shot / mechanism note shown in telemetry; steps may set it. */
    public String shotNote = "";
    int homeRetries = 0;                 // turret homes restarted after a FAULT
    private boolean started = false;

    // ---- mid-route relocalization (same state machine as the INIT start-pose refinement) ----
    private AutoStartRelocalization routeRelocalization;
    private double routeRelocalizeUntil;
    private MotionSample routeSeed;
    private boolean routeRelocalized;
    private String routeRelocalizeStatus = "Not requested";

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
        this(hw, drive, alliance, tagRelocalization, pollenCamera, () -> System.nanoTime() * 1e-9);
    }
    /** Explicit clock (seconds) for the robot and its timed subsystems; desktop tests use simulated time. */
    PlannerRobot(HardwareMap hw, PedroDrive drive, TurretSubsystem.Alliance alliance,
                 boolean tagRelocalization, boolean pollenCamera, DoubleSupplier clock) {
        this.drive = drive;
        this.alliance = alliance;
        this.clock = clock;
        intake = new IntakeSubsystem(hw);        // never resets intake_motor's encoder (turret encoder channel)
        transfer = new TransferSubsystem(hw);
        gate = new GateSubsystem(hw, clock, CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS);
        // Starts non-blocking turret homing and puts the hood at home.
        turret = new TurretSubsystem(hw, tagRelocalization, clock);
        turret.setShooterController(CompetitionShotPipeline.FLYWHEEL_CONTROLLER);   // TeleOp's flywheel control
        turret.setEncoderCorrectionEnabled(true);
        this.tagRelocalization = tagRelocalization && turret.hasLimelight();
        if (tagRelocalization && !this.tagRelocalization) missing.add("limelight (tag relocalization off)");
        // TeleOp's firing (continuous competition feed) with a fixed route target; camera corrections when relocalizing.
        shots = new ShotCoordinator(turret, TeleOpFieldReference.target(alliance, TurretSubsystem.HiveSide.AUDIENCE),
                ShootingConfiguration.MOUNT, ShootingConfiguration.PROFILE, CompetitionFeedController.firing(),
                this.tagRelocalization ? ShotCoordinator.AimMode.AUTONOMOUS_RELOCALIZING
                        : ShotCoordinator.AimMode.AUTONOMOUS_ODOMETRY);
        firing = shots.firing();
        depollinator = optional("depollinator", () -> new DepollinatorSubsystem(hw, clock));  // homes at INIT
        slide = optional("linear slide", () -> new LinearSlideSubsystem(hw, clock));         // must be fully down at INIT
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

    private double now() { return clock.getAsDouble(); }
    static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    /** INIT alliance select changed: target the new alliance and restart the start-pose refinement. */
    public void setAlliance(TurretSubsystem.Alliance next) {
        if (next == alliance) return;
        alliance = next;
        shots.cancelFieldRehomeRefinement("alliance changed");
        shots.setFollowNearestCell(false);
        shots.select(TeleOpFieldReference.target(next, TurretSubsystem.HiveSide.AUDIENCE));
        shots.invalidateReference(now());
        hasAimPoint = false;
        relocalization = new AutoStartRelocalization();
    }

    /**
     * Hive cell for a cell named in the planned alliance's frame (AUDIENCE / FAR as drawn): under the INIT
     * alliance point mirror the two cells swap, like Zenith's {@code aim(frame=AUTHORED)}.
     */
    public static TurretSubsystem.HiveSide authoredSide(TurretSubsystem.HiveSide side, boolean mirrored) {
        return AutonomousCommands.transformSide(side, AutonomousCommands.TargetFrame.AUTHORED, mirrored ? Math.PI : 0);
    }

    /**
     * Shot target for a field point (inches, after the alliance mirror). Within {@link #CELL_MATCH_INCHES} of a hive
     * opening this is TeleOp's exact cell target; otherwise a custom point that keeps the nearest cell's tags.
     */
    public static GoalTarget targetFor(Pose fieldPointInches) {
        double x = fieldPointInches.x() * METERS_PER_INCH, y = fieldPointInches.y() * METERS_PER_INCH;
        TurretSubsystem.Alliance a = x < 0 ? TurretSubsystem.Alliance.RED : TurretSubsystem.Alliance.BLUE;
        TurretSubsystem.HiveSide s = y < 0 ? TurretSubsystem.HiveSide.AUDIENCE : TurretSubsystem.HiveSide.FAR;
        GoalTarget cell = TeleOpFieldReference.target(a, s);
        if (Math.hypot(cell.x - x, cell.y - y) <= CELL_MATCH_INCHES * METERS_PER_INCH) return cell;
        return new GoalTarget(a, s, GoalTarget.HiveState.RAISED, x, y, cell.height, true, cell.geometryId,
                cell.tags.values().toArray(new GoalTarget.Tag[0]));
    }

    private void selectTarget(GoalTarget next) {
        GoalTarget current = shots.target();
        if (current != null && current.identity().equals(next.identity()) && current.x == next.x && current.y == next.y)
            return;
        shots.select(next);
        firing.stop(now());   // TeleOp rule: a different cell must earn its own readiness interval
    }

    // =====================================================================================
    //  COMMANDS - call from planner steps, path callbacks and loop code
    // =====================================================================================
    /** Legacy: intake in (1, TeleOp automatic collection), out (-1) or off (0). */
    public void setIntake(int mode) {
        setIntake(mode > 0 ? IntakeMode.IN : mode < 0 ? IntakeMode.REVERSE : IntakeMode.OFF, 0, false);
    }
    /**
     * @param seconds     > 0: switch the intake off after this long (works while driving)
     * @param offWhenFull switch it off once the transfer confirms a full queue
     */
    public void setIntake(IntakeMode mode, double seconds, boolean offWhenFull) {
        intakeMode = mode;
        intakeOffAt = seconds > 0 ? now() + seconds : Double.POSITIVE_INFINITY;
        intakeOffWhenFull = offWhenFull;
        if (mode == IntakeMode.IN || mode == IntakeMode.FORCE) holdLoading = false;   // collecting loads stage 1
    }
    public IntakeMode intakeMode() { return intakeMode; }

    /** Legacy flywheel on (shot table or manual RPM, see setRpmAuto / setRpmManual) / off. */
    public void spin(boolean on) { flywheel = !on ? Flywheel.OFF : rpmAuto ? Flywheel.AUTO : Flywheel.MANUAL; }
    public void setRpmAuto() { rpmAuto = true; if (flywheel != Flywheel.OFF) flywheel = Flywheel.AUTO; }
    /** Flywheel output RPM, the same units as TeleOp and the shot table. */
    public void setRpmManual(double rpm) {
        rpmAuto = false; rpmManual = ShooterSpeed.clampFlywheelRpm(rpm);
        if (flywheel != Flywheel.OFF) flywheel = Flywheel.MANUAL;
    }
    /** TeleOp's idle flywheel speed ({@link CompetitionShotPipeline#FLYWHEEL_IDLE_RPM}). */
    public void setFlywheelIdle() { flywheel = Flywheel.IDLE; }
    public Flywheel flywheel() { return flywheel; }
    /** Spin-up only: flywheel RPM and hood from the shot table for the current target; the turret does not move. */
    public void spinUpAuto() { rpmAuto = true; flywheel = Flywheel.AUTO; hoodAuto = true; }

    public void setHoodAuto() { hoodAuto = true; }
    /** Hood travel down from home in degrees (0 = home / highest launch, 30 = lowest). */
    public void setHoodManual(double travelDeg) {
        hoodAuto = false;
        hoodManualDeg = clamp(travelDeg, TurretSubsystem.PITCH_MIN_DEG, TurretSubsystem.PITCH_MAX_DEG);
    }

    /** Aim / range target: a field point in inches (after the alliance mirror; see {@link #targetFor}). */
    public void aimAt(Pose fieldPointInches) {
        shots.setFollowNearestCell(false);
        selectTarget(targetFor(fieldPointInches));
        hasAimPoint = true;
    }
    /** TeleOp's target rule: this alliance's cell on the robot's half of the field, re-selected as it drives. */
    public void aimAtNearestCell() {
        MotionSample p = drive.sample();
        TurretSubsystem.HiveSide side = p == null ? null : TeleOpFieldReference.nearestSide(p.y, null);
        selectTarget(TeleOpFieldReference.target(alliance, side == null ? TurretSubsystem.HiveSide.AUDIENCE : side));
        shots.setFollowNearestCell(true);
        hasAimPoint = true;
    }
    /** Auto-aim trims, as TeleOp's GP2 trims (yaw deg left+, hood deg, RPM), clipped to AutoAimAdjustments' bounds. */
    public void setTrims(double yawDeg, double hoodDeg, double rpm) { trims.set(yawDeg, hoodDeg, rpm); }
    /** Aim only: the turret tracks the target (TeleOp RT yaw); hood and flywheel keep their settings. */
    public void setTracking(boolean on) { yawMode = on ? YawMode.TRACK : YawMode.HOLD; if (on) fixedHeadingDeg = Double.NaN; }
    /** Fixed turret heading in the Driver Hub frame: front 0, right (home) 90, back 180, left 270 deg. */
    public void setTurretHeading(double headingDeg) { yawMode = YawMode.FIXED; fixedHeadingDeg = headingDeg; }
    /** Hold the gate open (true) or let the feed control it again (false). */
    public void gateOpen(boolean open) { gateHeldOpen = open; }
    /** Pause stage-1 loading (true) or load until full like TeleOp (false). */
    public void holdTransfer(boolean hold) { holdLoading = hold; }

    /**
     * Volley: TeleOp's full auto-shoot. Aim at the point (null: TeleOp's nearest own cell), set hood / RPM from the
     * shot table, wait for readiness, feed for {@code feedSeconds}, then apply {@code after}.
     */
    public void startVolley(Pose aimPointInches, double feedSeconds, double readyTimeoutSeconds,
                            PlannerVolley.TimeoutAction onTimeout, After after) {
        if (aimPointInches == null) aimAtNearestCell(); else aimAt(aimPointInches);
        yawMode = YawMode.TRACK; fixedHeadingDeg = Double.NaN;
        hoodAuto = true; rpmAuto = true; flywheel = Flywheel.AUTO;
        startShot(true, feedSeconds, readyTimeoutSeconds, onTimeout, after);
    }
    /**
     * Shoot with the current aim / hood / flywheel settings. waitForReady: TeleOp RT + LT (needs auto aim);
     * false: TeleOp GP1 A, feed now with readiness bypassed.
     */
    public void startShot(boolean waitForReady, double feedSeconds, double readyTimeoutSeconds,
                          PlannerVolley.TimeoutAction onTimeout, After after) {
        this.after = after;
        shotNote = "";
        volley.start(now(), feedSeconds, readyTimeoutSeconds, onTimeout, waitForReady);
    }
    /** Legacy Shoot: the fallback auto's continuous feed for feedSeconds, readiness bypassed. */
    public void startShoot() {
        startShot(false, feedSeconds, readyTimeoutSeconds, PlannerVolley.TimeoutAction.FIRE, After.KEEP);
    }
    /** A volley / shot is aiming or feeding. */
    public boolean isShooting() { return volley.active(); }
    public boolean volleyDone() { return !volley.active(); }
    public boolean volleySkipped() { return volley.phase() == PlannerVolley.Phase.SKIPPED; }
    public PlannerVolley.Phase volleyPhase() { return volley.phase(); }

    private void finishShot() {
        shotNote = volley.note();
        if (after == After.KEEP) return;
        yawMode = YawMode.HOLD;
        if (hoodAuto) hoodManualDeg = turret.getTargetPitchDegrees();
        hoodAuto = false;
        flywheel = after == After.IDLE ? Flywheel.IDLE : Flywheel.OFF;
    }

    public boolean turretHomed() { return turret.isHomed(); }
    public boolean turretFault() { return turret.hasHomingFault(); }
    /** Confirmed full queue (TransferSubsystem: sensor HIGH for FULL_CONFIRM_TIME_SECONDS). */
    public boolean transferFull() { return transfer.isFull(); }
    public boolean queueFull() { return transfer.isFull(); }

    /** Restart turret homing (turret bumped, or before a precise shot). Feeding waits for the new home. */
    public void rehomeTurret() {
        volley.cancel("turret re-home");
        double t = now();
        turret.homeTurretAndZeroEncoder(t);
        shots.invalidateReference(t);
    }

    /** Turret on target: homed, and within TeleOp's yaw tolerance of the current bearing (or fixed heading). */
    public boolean turretReady() {
        if (!turret.isHomed()) return false;
        if (yawMode == YawMode.TRACK) {
            ShotPlanner.Solution s = shots.solution();
            return poseFresh && s.valid && ShotReadiness.angleReady(Math.toDegrees(s.yaw) - turret.getEncoderAngleDegrees());
        }
        return ShotReadiness.angleReady(turret.getTargetYawDegrees() - turret.getEncoderAngleDegrees());
    }
    public boolean turretOnTarget() { return turretReady(); }
    /** TeleOp's shot-ready check (ready light / RT rumble): turret on target, hood settled, flywheel at speed. */
    public boolean shotReady() { return pipeline.shotReady(); }
    /** Legacy name of {@link #shotReady()}. */
    public boolean ready() { return shotReady(); }
    /** Flywheel at its target and hood settled (spin-up only). */
    public boolean flywheelReady() { return turret.isShooterAtSpeed() && turret.isHoodTracking(); }
    public ShotPlanner.Solution shotSolution() { return shots.solution(); }

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

    // ---- status lights: TeleOp's colors, or one color on all three for a while ----
    public void showLights(RobotStatusLights.Color color, double seconds) {
        lightsColor = color; lightsUntil = now() + Math.max(0, seconds);
    }

    // ---- pollen collection: PollenNavigator drives the follower and intake until it finishes ----
    /** Start a camera-guided pollen collection; the planner step must not command the follower meanwhile. */
    public void startCollect() {
        if (vision == null) { note("pollen camera"); collectionStatus = "SKIPPED: pollen camera not configured"; return; }
        volley.cancel("pollen collection");
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
        intakeMode = IntakeMode.OFF;
    }
    public boolean pollenVisible() {
        return pollenSnapshot != null && pollenSnapshot.autonomousReady && pollenSnapshot.target != null;
    }

    // ---- field pose correction ----
    /** The INIT start-pose refinement or a Relocalize step wrote a tag-refined pose into Pinpoint. */
    public boolean relocalized() {
        return relocalization.state() == AutoStartRelocalization.State.REFINED || routeRelocalized;
    }
    /**
     * Relocalize mid-route (robot stopped): the INIT refinement again (FieldRehomeRefinement: 5 consistent hive-tag
     * frames, at most 0.15 m, translation only), written into Pinpoint on success; the pose stays otherwise.
     */
    public void startRelocalize(double timeoutSeconds) {
        routeRelocalized = false;
        if (!tagRelocalization) {
            routeRelocalization = null;
            routeRelocalizeStatus = "SKIPPED: tag relocalization off / no Limelight";
            return;
        }
        volley.cancel("relocalize");
        routeRelocalization = new AutoStartRelocalization();
        routeRelocalizeUntil = now() + Math.max(.1, timeoutSeconds);
        routeRelocalizeStatus = "WAITING";
    }
    public boolean relocalizeDone() { return routeRelocalization == null; }
    /**
     * TeleOp's corner rehome for autonomous: declare the robot is at a known field pose (squared against walls).
     * Written only to a READY Pinpoint with a fresh pose; returns false (and notes it) otherwise.
     */
    public boolean setFieldPose(Pose fieldPoseInches) {
        if (!drive.ready(now())) { shotNote = "SKIPPED field pose: Pinpoint not ready"; return false; }
        drive.setFieldPose(fieldPoseInches);
        return true;
    }

    private void updateRouteRelocalization(double now) {
        if (routeRelocalization == null) return;
        if (now > routeRelocalizeUntil) {
            shots.cancelFieldRehomeRefinement("planner relocalize timed out");
            routeRelocalizeStatus = "TIMEOUT (pose kept): " + routeRelocalization.status();
            routeRelocalization = null;
            return;
        }
        boolean poseReady = drive.ready(now);
        switch (routeRelocalization.update(turret.isHomed(), poseReady, shots.fieldRehomePending(), shots.fieldRehomeStatus())) {
            case REQUEST_REFINEMENT:
                routeSeed = drive.sample();
                shots.requestFieldRehomeRefinement(now, AutoStartRelocalization.landmarkFor(routeSeed.y));
                break;
            case APPLY_REFINED_POSE:
                MotionSample refined = shots.pose();
                drive.setFieldPose(BioBuzzRobot.pedroPose(refined));
                routeRelocalization.applied(Math.hypot(refined.x - routeSeed.x, refined.y - routeSeed.y) / METERS_PER_INCH);
                routeRelocalizeStatus = routeRelocalization.status();
                routeRelocalized = true;
                routeRelocalization = null;
                break;
            default:
                routeRelocalizeStatus = routeRelocalization.status();
                break;
        }
    }

    private void note(String device) { shotNote = "SKIPPED: " + device + " not configured"; }

    /** Mid-auto stop: intake, transfer, feed and flywheel off; the turret holds its last target and keeps its home. */
    public void stopAll() {
        stopCollect();
        volley.cancel("stop all");
        if (routeRelocalization != null) { shots.cancelFieldRehomeRefinement("stop all"); routeRelocalization = null; }
        intakeMode = IntakeMode.OFF; intakeOffAt = Double.POSITIVE_INFINITY; intakeOffWhenFull = false;
        flywheel = Flywheel.OFF;
        yawMode = YawMode.HOLD;
        fixedHeadingDeg = Double.NaN;
        gateHeldOpen = false;
        holdLoading = true;   // stage 1 stays off until an intake / transfer step resumes loading
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
     * turret encoder, homing and flywheel velocity, the shot coordinator's pose (with Limelight corrections) and
     * the pollen camera frame. A home that faults (turret bumped or still moving in its settle window) restarts at
     * once; otherwise the turret would stay unhomed and never aim for the rest of the auto.
     */
    public void readSensors(double now) { readSensors(now, drive.ready(now) ? drive.sample() : null); }
    /** {@code sample}: this loop's fresh, field-referenced Pinpoint sample, or null. */
    void readSensors(double now, MotionSample sample) {
        transfer.readSensors(now);
        turret.update(now);
        if (turret.hasHomingFault()) { homeRetries++; turret.homeTurretAndZeroEncoder(now); }
        shots.updatePose(now, sample);
        pollenSnapshot = vision == null ? null : vision.snapshot(System.nanoTime());
    }

    /**
     * INIT loop, robot stationary: turret homing travel and, with tag relocalization, the start-pose refinement
     * (BioBuzzRobot.relocalizeStart). Nothing spins or feeds before START; the slide stays unpowered.
     */
    public void initLoop(double now) {
        if (tagRelocalization) {
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
        shots.cancelFieldRehomeRefinement("START before the refinement finished");
    }

    /**
     * Once per loop, after the step and the drive update: the volley sequence, then TeleOp's shot pipeline (aim,
     * readiness, firing; gate, transfer and intake), then the slide and the status lights.
     */
    public void apply(double now) { apply(now, drive.ready(now) ? drive.sample() : null); }
    /** {@code sample}: this loop's fresh, field-referenced Pinpoint sample, or null. */
    void apply(double now, MotionSample sample) {
        if (now >= intakeOffAt || (intakeOffWhenFull && transfer.isFull())) {
            intakeMode = IntakeMode.OFF; intakeOffAt = Double.POSITIVE_INFINITY; intakeOffWhenFull = false;
        }
        updateRouteRelocalization(now);

        // Volley sequencing reads the firing state from its last update (previous loop).
        boolean wasActive = volley.active();
        volley.update(now, firing.feeding(), turret.isHomed());
        if (wasActive && volley.finished()) finishShot();

        // Fixed heading (TeleOp manual yaw, then hold) unless the camera refinement owns the turret.
        if (yawMode == YawMode.FIXED && Double.isFinite(fixedHeadingDeg) && turret.isHomed() && !shots.fieldRehomePending())
            turret.setTargetYawDegrees(turret.reachableYawTarget(TurretControl.robotHeadingToYawDegrees(fixedHeadingDeg)));
        boolean autoHood = hoodAuto && hasAimPoint;
        if (!autoHood) turret.setPitchAngleDegrees(hoodManualDeg);

        // ---- pollen collection owns the follower and intake while it runs ----
        double collectIntake = Double.NaN;
        if (collecting) {
            long nanos = System.nanoTime();
            MotionSample p = drive.sample();
            navigator.update(nanos, pollenSnapshot == null ? null : pollenSnapshot.target,
                    p.x * 1000, p.y * 1000, Math.toDegrees(p.heading),
                    drive.ready(now) && pollenSnapshot != null && pollenSnapshot.autonomousReady, transfer.isFull());
            collectIntake = navigator.getIntakePower();
            collectionStatus = navigator.getState() + ": " + navigator.getStatus();
            navigation.update(navigator, nanos);
        }
        IntakeMode intakeNow = Double.isNaN(collectIntake) ? intakeMode
                : collectIntake > 0 ? IntakeMode.IN : collectIntake < 0 ? IntakeMode.REVERSE : IntakeMode.OFF;

        // ---- TeleOp's shot pipeline ----
        boolean autoYaw = yawMode == YawMode.TRACK && hasAimPoint;
        boolean autoRpm = flywheel == Flywheel.AUTO && hasAimPoint;
        request.aim = autoYaw || autoHood || autoRpm;
        request.controlYaw = autoYaw;
        request.controlHood = autoHood;
        request.controlRpm = autoRpm;
        request.automaticFeed = volley.automaticFeed();
        request.manualFeed = volley.manualFeed();
        request.inhibit = false;
        request.manualRpm = flywheel == Flywheel.OFF ? 0 : flywheel == Flywheel.MANUAL ? rpmManual
                : CompetitionShotPipeline.FLYWHEEL_IDLE_RPM;
        request.intakeForward = intakeNow == IntakeMode.FORCE;
        request.intakeReverse = intakeNow == IntakeMode.REVERSE;
        request.intakeAutomatic = intakeNow == IntakeMode.IN;
        request.intakePowerScale = intakePowerScale;
        request.holdLoading = holdLoading;
        request.gateHeldOpen = gateHeldOpen;
        pipeline.beginLoop(now, firing);
        pipeline.aim(now, turret, shots, sample, trims, request);
        turret.applyYawCommand(now); // Same-loop servo write; also drives homing travel.
        pipeline.feed(now, firing, gate, transfer, intake, request, false);
        MotionSample aimPose = shots.pose();
        poseFresh = aimPose != null && aimPose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS);

        if (slide != null) { if (started) slide.update(); else slide.trackWithoutDriving(); }

        // ---- status lights: TeleOp's colors (or a Lights step color); a new fault blinks all three red first ----
        showFaultAlert(now);
        if (now < lightsUntil) {
            if (!lights.isFaultAlerting()) for (int i = 0; i < 3; i++) lights.set(i, lightsColor);
        } else {
            lights.showIntake(transfer.isFull(), request.intakeForward, request.intakeReverse, intake.getCommandedPower());
            lights.showShooter(now, firing.state(), request.manualFeed, pipeline.shotReady(), turret.getTargetShooterRpm() > 0);
            showReference(now);
        }
    }

    private void showFaultAlert(double now) {
        lights.showFaultAlert(now, turret.hasHomingFault(), slideFault(),
                firing.state() == FiringCoordinator.State.FAULT);
    }

    private void showReference(double now) {
        MotionSample p = shots.pose() != null ? shots.pose() : drive.sample();
        lights.showReference(!turret.hasHomingFault(), turret.isHomed(),
                p != null && p.fieldReferenced && p.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS));
    }

    public void addTelemetry(Telemetry t) {
        turret.addDriverTelemetry(t);
        transfer.addDriverTelemetry(t);
        if (homeRetries > 0) t.addData("Turret home retries", homeRetries);
        t.addData("Turret target heading (deg CW)", "%.1f | %s | %s", turret.getTargetRobotHeadingDegrees(), yawMode,
                !turret.isHomed() ? "NOT HOMED" : turretReady() ? "ON TARGET" : yawMode == YawMode.TRACK && !poseFresh
                        ? "POSE STALE" : "MOVING");
        ShotPlanner.Solution s = shots.solution();
        t.addData("Shot target", "%s | %s", hasAimPoint ? shots.target().alliance + " " + shots.target().side
                + (shots.followsNearestCell() ? " (nearest)" : "") : "none (TeleOp idle settings)", s.reason);
        t.addData("Aim range (in) / hood (deg) / RPM", s.valid ? String.format(Locale.US, "%.1f / %.1f / %.0f",
                s.effectiveRange / METERS_PER_INCH, s.hoodDeg, s.rpm) : "no solution");
        t.addData("Flywheel / hood", "%s %.0f RPM target | hood %s %.1f deg", flywheel, turret.getTargetShooterRpm(),
                hoodAuto && hasAimPoint ? "AUTO" : "MANUAL", turret.getTargetPitchDegrees());
        t.addData("Shot", "%s | ready %s | feed %s: %s%s", volley.phase(), shotReady(), firing.state(), firing.reason(),
                shotNote.isEmpty() ? "" : " | " + shotNote);
        t.addData("Intake / transfer", "%s | %s%s", intakeMode, holdLoading ? "HOLD" : "LOAD", gateHeldOpen ? " | gate held open" : "");
        if (tagRelocalization) t.addData("Tag relocalization", "%s | %s", relocalization.status(), shots.visionDecision());
        if (!"Not requested".equals(routeRelocalizeStatus)) t.addData("Route relocalize", routeRelocalizeStatus);
        if (tagRelocalization && relocalization.checkPlacement()) t.addData("CHECK PLACEMENT",
                "hive tags disagree with the authored start pose by more than %.0f in",
                FieldRehomeRefinement.MAX_CORRECTION_METERS * PedroConfiguration.INCHES_PER_METER);
        if (depollinator != null) depollinator.addTelemetry(t);
        if (slide != null) slide.addDriverTelemetry(t);
        if (vision != null) t.addData("Pollen collection", collectionStatus);
        if (!missing.isEmpty()) t.addData("Not configured (steps skipped)", String.join(", ", missing));
        lights.addWarningTelemetry(t);
    }
}
