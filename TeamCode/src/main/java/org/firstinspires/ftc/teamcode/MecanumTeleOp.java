package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Layer 3 Combined TeleOp: Coordinates {@link MecanumDrivetrain}, {@link PinpointOdometry},
 * {@link IntakeSubsystem}, {@link TransferSubsystem}, {@link GateSubsystem}, {@link DepollinatorSubsystem},
 * and {@link TurretSubsystem} (rotation servos on ports 1 & 2, 108:40 REV Through-Bore Encoder
 * on Expansion Hub quad port 1 via intake_motor, 2:1 pitch servo on port 0 from 0-30 deg downward travel from the offset hood home (+15 to -15 deg launch elevation), 6000 RPM flywheel motor on port 0,
 * and Limelight 3A pitched ShootingConfiguration.CAMERA_PITCH_UP_DEG toward the goal).
 *
 * <p>Controls (Gamepad 1 — Driver + Turret Assist, or Gamepad 2 — Dedicated Turret Operator):
 * <ul>
 *     <li><b>Gamepad 1 Left Stick</b>: Forward / Backward & Strafe translation</li>
 *     <li><b>Gamepad 1 Right Stick X</b>: Chassis rotation; capped to a turret-trackable rate while RT auto-aims</li>
 *     <li><b>Gamepad 1 Right / Left Stick Click</b>: Deploy depollinator 135 deg CW / return to 30 deg home</li>
 *     <li><b>Gamepad 1 Right Trigger (Hold)</b>: Prepare auto-aim, hood angle and flywheel RPM</li>
 *     <li><b>Gamepad 1 Right + Left Triggers (Hold)</b>: Start automatic shooting when ready</li>
 *     <li><b>Gamepad 1 A (Hold)</b>: Manually feed immediately (continuous feed, readiness bypassed)</li>
 *     <li><b>Gamepad 2 Right Trigger (Hold)</b>: Manually feed immediately (continuous feed, readiness bypassed)</li>
 *     <li><b>Gamepad 2 Left Trigger (Hold)</b>: Force gate closed and stop the transfer (both motors); overrides firing</li>
 *     <li><b>Feed mode</b>: Always CONTINUOUS (staggered mode and its GP2 B toggle removed 2026-11-19)</li>
 *     <li><b>Gamepad 2 D-Pad Right (Press)</b>: Clear auto yaw/hood/RPM offsets</li>
 *     <li><b>Gamepad 2 D-Pad Left (Press)</b>: Set field pose at the alliance corner opposite its pollen depot
 *         (also replaces a not-yet-applied autonomous pose). Written only to a settled READY Pinpoint: an early
 *         INIT press waits for the IMU recalibration; after START an unsettled press is refused
 *         ({@link CornerRehomeRequest})</li>
 *     <li><b>Autonomous pose handoff</b>: in INIT, a field pose saved by this session's autonomous (same alliance,
 *         at most {@link AutoPoseHandoff#MAX_AGE_SECONDS} old) is applied once the Pinpoint settles, so auto aim
 *         works from the first loop; see {@link AutoPoseHandoff}</li>
 *     <li><b>Gamepad 2 X (Press)</b>: Rehome turret only</li>
 *     <li><b>Gamepad 1/2 Right Bumper</b>: Force intake and stage-1 loading forward, overriding full</li>
 *     <li><b>Gamepad 1/2 Left Bumper</b>: Reverse intake only and pause stage-1 loading; both bumpers stop collection</li>
 *     <li><b>Flywheel</b>: Idles at {@link #FLYWHEEL_IDLE_RPM} (3000) output RPM from START; GP2 Y/A adjusts this
 *         manual RPM, auto-aim overrides it while RT is held.
 *         Regulated by bang-bang + feedforward fusion ({@link #FLYWHEEL_CONTROLLER})</li>
 *     <li><b>Transfer</b>: Stage 1 (both linked motors) loads until full; a shot opens the gate and feeds stage 1
 *         together, with no gate delay, for as long as the feed is held</li>
 *     <li><b>Full Queue</b>: After 0.5 seconds HIGH, intake stops; stage 1 runs on 0.5 seconds. LOW resumes loading automatically.</li>
 *     <li><b>Intake</b>: Automatically runs forward with space; bumpers always override direction</li>
 *     <li><b>Gamepad 2 Right Stick X</b>: Proportional clockwise turret speed, full stick = rated 6 V speed;
 *         release holds measured heading. Front=0, right/home=90, back=180, left=270 deg (265..275 excluded).</li>
 *     <li><b>Gamepad 2 D-Pad Up / Down</b>: Lower/raise launch elevation by 0.5 deg within -15..+15 deg; while auto aim
 *         sets the hood (calibrated profile), adjusts a bounded hood offset instead</li>
 *     <li><b>Gamepad 2 Y / A</b>: Increase / decrease manual flywheel RPM by 50; while auto aim sets the RPM
 *         (calibrated profile), adjusts a bounded RPM offset instead</li>
 *     <li><b>Gamepad 2 Back (Press)</b>: Show / hide full diagnostics telemetry below the short driver view</li>
 *     <li><b>Gamepad 2 Left Stick Click (Press)</b>: Toggle linear slide fully down / fully up
 *         ({@link LinearSlideSubsystem}; slide must be down at INIT)</li>
<<<<<<< Updated upstream
 *     <li><b>Gamepad 1 B (Press)</b>: Toggle slide claw open / closed ({@link SlideClawSubsystem};
 *         no effect with the shot-blocker end effector)</li>
 *     <li><b>Rumble</b> ({@link DriverRumble}): both gamepads blip twice at 20 s left and rumble 1 s at 5 s left
 *         (clock from TeleOp START); Gamepad 1 buzzes briefly when automatic shooting becomes READY
 *         and rumbles 0.4 s when the transfer confirms a full queue</li>
=======
 *     <li><b>GP1 rumble</b> ({@link DriverRumble}): two pulses when the transfer queue becomes full; a steady
 *         rumble while RT is held until the shot is ready</li>
 *     <li><b>Fault lights</b>: a new turret homing, slide, Pinpoint or firing fault blinks all three status
 *         lights red for {@link RobotStatusLights#FAULT_ALERT_SECONDS} s</li>
>>>>>>> Stashed changes
 * </ul>
 */
public abstract class MecanumTeleOp extends LinearOpMode {

    protected abstract TurretSubsystem.Alliance getAlliance();

    /** Rear and side against the corner walls, chassis pointing inward toward the alliance goal. */
    protected abstract Pose2D getFieldRehomePose();

    /** Default robot configuration; desktop fixtures can inject explicitly synthetic shot measurements. */
    protected ShotCoordinator createShotCoordinator(TurretSubsystem turret) {
        return new ShotCoordinator(turret,
                TeleOpFieldReference.target(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE),
                ShotCoordinator.AimMode.TELEOP_ODOMETRY);
    }

    private static final double PITCH_STEP_DEG = 0.5;
    private static final double SHOOTER_RPM_STEP = 50.0;
    private static final double TRIGGER_DEADBAND = 0.1;
    /** Manual/idle flywheel speed from START (3000 since 2026-11-19, was 3500); GP2 Y/A adjusts it. */
    public static final double FLYWHEEL_IDLE_RPM = 3000.0;
    /** Both competition TeleOps regulate the flywheel with bang-bang + feedforward fusion (untuned gains). */
    public static final TurretSubsystem.ShooterController FLYWHEEL_CONTROLLER =
            TurretSubsystem.ShooterController.BANG_BANG_FUSION;

    private void rehomeField(PinpointOdometry odometry, ShotCoordinator shots, Pose2D cornerPose) {
        odometry.setPose(cornerPose);
        if (shots != null) shots.requestFieldRehomeRefinement(System.nanoTime() * 1e-9);
    }

    /** Any new fault blinks all lights red; call before the other light updates (firing is null in INIT). */
    private static void showFaultAlert(RobotStatusLights lights, double now, TurretSubsystem turret,
                                       LinearSlideSubsystem slide, PinpointOdometry odometry,
                                       FiringCoordinator firing) {
        lights.showFaultAlert(now,
                turret != null && turret.hasHomingFault(),
                slide != null && slide.getState() == LinearSlideController.State.FAULT,
                odometry.hasDeviceFault(),
                firing != null && firing.state() == FiringCoordinator.State.FAULT);
    }

    @Override
    public void runOpMode() {
        final Pose2D fieldRehomePose = getFieldRehomePose();
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        PinpointOdometry odometry;
        IntakeSubsystem intake = null;
        GateSubsystem gate = null;
        TransferSubsystem transfer = null;
        DepollinatorSubsystem depollinator;
        TurretSubsystem turret = null;
        RobotStatusLights lights = null;
        LinearSlideSubsystem slide = null;
        try {
            odometry = new PinpointOdometry(hardwareMap);
            intake = new IntakeSubsystem(hardwareMap);
            gate = new GateSubsystem(hardwareMap, CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS);
            transfer = new TransferSubsystem(hardwareMap);
            depollinator = new DepollinatorSubsystem(hardwareMap);
            lights = new RobotStatusLights(hardwareMap);
            String turretError = null;
            try {
                turret = new TurretSubsystem(hardwareMap);
                turret.setShooterController(FLYWHEEL_CONTROLLER);
                turret.setEncoderCorrectionEnabled(true);
                turret.setTargetHiveCell(getAlliance(), TurretSubsystem.HiveSide.AUDIENCE);
            } catch (Exception e) {
                turretError = e.getMessage();
            }
            // Optional like the turret: an unwired slide must not stop the match OpMode.
            String slideError = null;
            try {
                slide = new LinearSlideSubsystem(hardwareMap);
            } catch (Exception e) {
                slideError = e.getMessage();
            }

            ShotCoordinator shots = turret == null ? null : createShotCoordinator(turret);
            boolean showDiagnostics = false;
            boolean previousDiagnosticsToggle = false;

            // Homing continues through INIT and an early START without blocking the drivetrain.
            boolean previousHome = false;
            boolean previousFieldRehome = false;
            // This session's autonomous pose (same alliance, recent) is applied once the Pinpoint has settled.
            AutoPoseHandoff.Pending autoPose = AutoPoseHandoff.take(getAlliance(), System.nanoTime() * 1e-9);
            // The corner pose also waits for a settled READY Pinpoint: INIT recalibrates its IMU.
            CornerRehomeRequest cornerRehome = new CornerRehomeRequest();
            while (opModeInInit()) {
                loop.begin();
                boolean pinpointSettled = odometry.isSettled(System.nanoTime() * 1e-9); // From last loop's read.
                if (gamepad2.dpad_left && !previousFieldRehome) {
                    autoPose.cancel("Replaced by GP2 D-pad Left corner rehome");
                    if (cornerRehome.press(pinpointSettled, true)) rehomeField(odometry,shots,fieldRehomePose);
                } else if (cornerRehome.poll(pinpointSettled)) rehomeField(odometry,shots,fieldRehomePose);
                previousFieldRehome = gamepad2.dpad_left;
                // Placement before the single Pinpoint read, like the corner rehome; uses last loop's status.
                Pose2D handoffPose = autoPose.poll(System.nanoTime() * 1e-9,
                        odometry.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY);
                if (handoffPose != null) odometry.setPose(handoffPose);
                if (gamepad2.back && !previousDiagnosticsToggle) showDiagnostics = !showDiagnostics;
                previousDiagnosticsToggle = gamepad2.back;
                odometry.update();
                if (slide != null) slide.trackWithoutDriving();
                if (turret != null) {
                    if (gamepad2.x && !previousHome) {
                        shots.cancelFieldRehomeRefinement("turret home requested");
                        turret.homeTurretAndZeroEncoder();
                    }
                    if (Math.abs(gamepad2.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND)
                        shots.cancelFieldRehomeRefinement("manual turret control");
                }
                previousHome = gamepad2.x;
                double now = System.nanoTime() * 1e-9;
                if (turret != null) {
                    turret.update();
                    shots.updatePose(now,odometry.getSample());
                    turret.applyYawCommand(); // Corner refinement may have set this loop's yaw.
                }
                MotionSample initPose = shots == null ? null : shots.pose();
                showFaultAlert(lights, now, turret, slide, odometry, null);
                lights.showReference(turret != null && !turret.hasHomingFault(), turret != null && turret.isHomed(),
                        initPose != null && initPose.fieldReferenced && initPose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS));

                telemetry.addData("Alliance", getAlliance());
                if (turret != null) turret.addDriverTelemetry(telemetry);
                else telemetry.addData("Turret", "UNAVAILABLE: %s", turretError);
                if (shots != null) shots.addDriverTelemetry(telemetry, now, false);
                odometry.addDriverTelemetry(telemetry);
                telemetry.addData("Auto pose handoff", autoPose.status());
                if (!CornerRehomeRequest.NOT_REQUESTED.equals(cornerRehome.status()))
                    telemetry.addData("Corner rehome (GP2 D-pad Left)", cornerRehome.status());
                if (slide != null) slide.addDriverTelemetry(telemetry);
                else telemetry.addData("Slide (GP2 RS/LS)", "UNAVAILABLE: %s", slideError);
                lights.addWarningTelemetry(telemetry);
                telemetry.addLine("GP2: X = home turret | D-Pad Left = corner rehome | Back = diagnostics");
                telemetry.addLine("Slide must be fully down now (INIT = slide zero).");
                if (showDiagnostics) {
                    telemetry.addLine("--- DIAGNOSTICS ---");
                    drivetrain.addConfigurationTelemetry(telemetry);
                    odometry.addInitTelemetry(telemetry);
                    odometry.addTelemetry(telemetry);
                    if (shots != null) shots.addTelemetry(telemetry, now);
                    if (turret != null) turret.addTelemetry(telemetry);
                    transfer.addTelemetry(telemetry);
                    if (slide != null) slide.addTelemetry(telemetry);
                    lights.addTelemetry(telemetry);
                    loop.addTelemetry(telemetry);
                }
                telemetry.update(); idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            // The robot may move from here on, so a pose not yet applied is no longer known to be valid.
            autoPose.cancel("Not applied: Pinpoint did not settle before START");
            cornerRehome.cancel("Not applied: Pinpoint did not settle before START; press again");
            DriverRumble rumble = new DriverRumble(); // GP1: queue full; RT held until the shot is ready.

            FiringCoordinator fallbackFiring = CompetitionFeedController.firing();
            FiringCoordinator firing = shots == null ? fallbackFiring : shots.firing();
            CompetitionFeedController feedControl = new CompetitionFeedController();
            boolean previousDpadUp = false;
            boolean previousDpadDown = false;
            boolean previousRpmUp = false;
            boolean previousRpmDown = false;
            boolean previousTrimReset = gamepad2.dpad_right;
            AutoAimAdjustments aimAdjustments = new AutoAimAdjustments();
            double manualShooterRpm = FLYWHEEL_IDLE_RPM;
            IntakeQueueController intakeControl = new IntakeQueueController();
            boolean previousDepollinatorDeploy = false;
            boolean previousDepollinatorHome = false;
            // Held through START must not toggle on the first loop.
            boolean previousSlideExtend = gamepad2.right_stick_button;
            boolean previousSlideHome = gamepad2.left_stick_button;

            double previousLoopTime = System.nanoTime() * 1e-9;
            DriverRumble rumbleCues = new DriverRumble();
            rumbleCues.start(previousLoopTime);
            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                double dt = Math.max(0, Math.min(.1, now - previousLoopTime)); previousLoopTime = now;
                // 1. Update Pinpoint Odometry Computer
                // Apply placement before the single bulk read; aiming requalifies after velocity history resumes.
                // After START an unsettled Pinpoint refuses the press (the robot may be moving), never queues it.
                if (gamepad2.dpad_left && !previousFieldRehome
                        && cornerRehome.press(odometry.isSettled(now), false)) rehomeField(odometry,shots,fieldRehomePose);
                previousFieldRehome = gamepad2.dpad_left;
                odometry.update();
                if (gamepad2.back && !previousDiagnosticsToggle) showDiagnostics = !showDiagnostics;
                previousDiagnosticsToggle = gamepad2.back;

                // 2. Robot-centric Mecanum Drive on Controller 1
                double axial = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                boolean autoAimHeld = gamepad1.right_trigger > TRIGGER_DEADBAND;
                double yaw = gamepad1.right_stick_x; // Full rotation speed, also while auto aiming.

                drivetrain.driveRobotCentric(axial, lateral, yaw);
                boolean deployDepollinator = gamepad1.right_stick_button;
                boolean homeDepollinator = gamepad1.left_stick_button;
                if (homeDepollinator && !previousDepollinatorHome) {
                    depollinator.home();
                } else if (deployDepollinator && !previousDepollinatorDeploy && !homeDepollinator) {
                    depollinator.deploy();
                }
                previousDepollinatorDeploy = deployDepollinator;
                previousDepollinatorHome = homeDepollinator;
                // Linear slide (shot blocker permanently attached) on GP2.
                boolean slideExtend = gamepad2.right_stick_button;
                boolean slideHome = gamepad2.left_stick_button;
                if (slide != null) {
                    if (slideHome && !previousSlideHome) slide.lower();
                    else if (slideExtend && !previousSlideExtend && !slideHome) slide.raise();
                    slide.update();
                }
                previousSlideExtend = slideExtend;
                previousSlideHome = slideHome;
                boolean automaticFeedHeld = autoAimHeld && gamepad1.left_trigger > TRIGGER_DEADBAND;
                boolean manualFeedHeld = gamepad1.a
                        || gamepad2.right_trigger > TRIGGER_DEADBAND;
                boolean inhibitFeedHeld = gamepad2.left_trigger > TRIGGER_DEADBAND;
                boolean intakeForwardHeld = gamepad1.right_bumper || gamepad2.right_bumper;
                boolean intakeReverseHeld = gamepad1.left_bumper || gamepad2.left_bumper;
                boolean autoShotReady = false;
                feedControl.updateSelection(now, firing);

                // 3. Turret & Limelight 3A Control (supports both Gamepad 1 and Gamepad 2)
                if (turret != null) {
                    boolean currentDpadUp = gamepad2.dpad_up;
                    boolean currentDpadDown = gamepad2.dpad_down;
                    boolean currentRpmUp = gamepad2.y;
                    boolean currentRpmDown = gamepad2.a;
                    if (gamepad2.dpad_right && !previousTrimReset) aimAdjustments.reset();
                    previousTrimReset = gamepad2.dpad_right;
                    int hoodSteps = (currentDpadUp && !previousDpadUp ? 1 : 0) - (currentDpadDown && !previousDpadDown ? 1 : 0);
                    int rpmSteps = (currentRpmUp && !previousRpmUp ? 1 : 0) - (currentRpmDown && !previousRpmDown ? 1 : 0);
                    // Hood/RPM trims apply only while auto aim actually sets hood and RPM (a calibrated solution,
                    // from the previous loop). Otherwise the shot still uses the manual hood/RPM, so the buttons
                    // keep adjusting those instead of hidden trims. RS-X trims yaw whenever RT is held.
                    boolean autoShotSettings = autoAimHeld && shots.automaticHoodRpmAvailable();
                    if (autoAimHeld) aimAdjustments.adjust(gamepad2.right_stick_x, dt,
                            autoShotSettings ? hoodSteps : 0, autoShotSettings ? rpmSteps : 0);
                    if (!autoShotSettings) {
                        if (hoodSteps != 0) turret.adjustPitchAngleDegrees(hoodSteps * PITCH_STEP_DEG);
                        manualShooterRpm = Math.max(0.0, Math.min(
                                TurretSubsystem.SHOOTER_MAX_RPM, manualShooterRpm + rpmSteps * SHOOTER_RPM_STEP));
                    }
                    previousDpadUp = currentDpadUp;
                    previousDpadDown = currentDpadDown;
                    previousRpmUp = currentRpmUp;
                    previousRpmDown = currentRpmDown;

                    if (!autoAimHeld && gamepad2.x && !previousHome) {
                        aimAdjustments.reset();
                        shots.cancelFieldRehomeRefinement("turret home requested");
                        turret.homeTurretAndZeroEncoder();
                    }
                    if (Math.abs(gamepad2.right_stick_x) > TurretControl.MANUAL_STICK_DEADBAND)
                        shots.cancelFieldRehomeRefinement("manual turret control");
                    previousHome = gamepad2.x;
                    turret.update();
                    turret.updateCompetitionManualYaw(gamepad2.right_stick_x, dt, autoAimHeld || shots.fieldRehomePending());

                    if (!autoAimHeld) {
                        turret.setShooterRpm(manualShooterRpm);
                    }
                    now = System.nanoTime() * 1e-9;
                    shots.setAutoAimAdjustments(now, aimAdjustments);
                    shots.update(now, odometry.getSample(), autoAimHeld, automaticFeedHeld,
                            manualFeedHeld, inhibitFeedHeld, false);
                    if (autoAimHeld && !shots.solution().aimable) turret.setShooterRpm(manualShooterRpm);
                    // Write this loop's yaw target now, not on the next loop's update().
                    turret.applyYawCommand();
                    autoShotReady = shots.automaticTrackingReady();
                }

                boolean openGate = shots != null && shots.firing().gateOpen();
                // Manual feed also works when turret hardware is unavailable.
                if (shots == null) {
                    fallbackFiring.update(now, false, manualFeedHeld, inhibitFeedHeld, false, false, false, false, "Turret unavailable");
                    openGate = fallbackFiring.gateOpen();
                }
                boolean feedStage1 = feedControl.feedStage1(now, firing);
                // Never true in continuous mode; kept so the transfer logic matches the shared coordinator.
                boolean closedGateStage1 = feedControl.closedGateStage1(firing)
                        && !inhibitFeedHeld && !intakeReverseHeld;
                gate.setOpen(openGate);
                boolean shootingBusy = automaticFeedHeld || manualFeedHeld || openGate
                        || firing.state() == FiringCoordinator.State.RECOVERY;
                transfer.update(now, feedControl.feedTransfers(firing, gate.isOpenForFeeding()),
                        !closedGateStage1 && (inhibitFeedHeld || shootingBusy || intakeReverseHeld),
                        feedStage1, closedGateStage1 || (intakeForwardHeld && !intakeReverseHeld));
                double intakePower = intakeControl.update(transfer.isFull(),
                        intakeForwardHeld, intakeReverseHeld, inhibitFeedHeld || shootingBusy);
                intake.setIntakePower(intakePower);
                showFaultAlert(lights, now, turret, slide, odometry, firing);
                lights.showIntake(transfer.isFull(), intakeForwardHeld, intakeReverseHeld,
                        intake.getCommandedPower());
                lights.showShooter(now, firing.state(), manualFeedHeld, autoShotReady,
                        turret != null && turret.getTargetShooterRpm() > 0);
                MotionSample aimingPose = shots == null ? null : shots.pose();
                lights.showReference(turret != null && !turret.hasHomingFault(), turret != null && turret.isHomed(),
                        aimingPose != null && aimingPose.fieldReferenced && aimingPose.fresh(now, ShotPlanner.MAX_SAMPLE_AGE_SECONDS));
<<<<<<< Updated upstream
                rumbleCues.update(now, autoShotReady, transfer.isFull());
                if (rumbleCues.endgameWarning()) {
                    rumbleBlips(gamepad1, DriverRumble.ENDGAME_WARNING_BLIPS);
                    rumbleBlips(gamepad2, DriverRumble.ENDGAME_WARNING_BLIPS);
                }
                if (rumbleCues.finalWarning()) {
                    rumble(gamepad1, DriverRumble.FINAL_WARNING_RUMBLE_MS);
                    rumble(gamepad2, DriverRumble.FINAL_WARNING_RUMBLE_MS);
                }
                if (rumbleCues.shotReady()) rumble(gamepad1, DriverRumble.SHOT_READY_RUMBLE_MS);
                else if (rumbleCues.transferFull()) rumble(gamepad1, DriverRumble.TRANSFER_FULL_RUMBLE_MS);
=======
                rumble.update(now, transfer.isFull(), autoAimHeld, autoShotReady, gamepad1);
>>>>>>> Stashed changes
                // 4. Driver view (most important first), optional diagnostics below, one flush per loop.
                FiringCoordinator.State firingState = firing.state();
                String shotStatus = inhibitFeedHeld ? "BLOCKED (GP2 LT)" : manualFeedHeld ? "MANUAL FEED"
                        : !autoAimHeld ? "OFF"
                        : !automaticFeedHeld ? "AIMING (hold LT to fire)"
                        : autoShotReady ? "READY" : "WAITING FOR AIM / HOOD / RPM";
                if (firingState == FiringCoordinator.State.INHIBITED || firingState == FiringCoordinator.State.FAULT)
                    telemetry.addData("Shot", "%s | %s: %s", shotStatus, firingState, firing.reason());
                else telemetry.addData("Shot", "%s | %s", shotStatus, firingState);
                if (turret != null) turret.addDriverTelemetry(telemetry);
                else telemetry.addData("Turret", "UNAVAILABLE: %s", turretError);
                if (shots != null) shots.addDriverTelemetry(telemetry, now, autoAimHeld);
                else telemetry.addData("Field rehome refinement", "CORNER ONLY: turret/camera unavailable");
                odometry.addDriverTelemetry(telemetry);
                if (cornerRehome.needsAttention())
                    telemetry.addData("Corner rehome (GP2 D-pad Left)", cornerRehome.status());
                transfer.addDriverTelemetry(telemetry);
                telemetry.addData("Collection", "%s | intake %+.1f", intakeReverseHeld
                                ? intakeForwardHeld ? "STOPPED" : "REVERSE"
                                : intakeForwardHeld ? "FORCED FORWARD" : "AUTO",
                        intake.getCommandedPower());
                telemetry.addData("Depollinator", depollinator.isDeployedCommanded() ? "DEPLOYED" : "HOME");
                if (slide != null) slide.addDriverTelemetry(telemetry);
                else telemetry.addData("Slide (GP2 RS/LS)", "UNAVAILABLE: %s", slideError);
                lights.addWarningTelemetry(telemetry);
                loop.addRateTelemetry(telemetry);
                if (showDiagnostics) {
                    telemetry.addLine("--- DIAGNOSTICS (GP2 Back to hide) ---");
                    transfer.addStatusTelemetry(telemetry);
                    if (shots != null) shots.addTelemetry(telemetry, now);
                    odometry.addTelemetry(telemetry);
                    telemetry.addData("Auto pose handoff", autoPose.status());
                    drivetrain.addTelemetry(telemetry);
                    intake.addTelemetry(telemetry);
                    gate.addTelemetry(telemetry);
                    transfer.addTelemetry(telemetry);
                    depollinator.addTelemetry(telemetry);
                    if (slide != null) slide.addTelemetry(telemetry);
                    lights.addTelemetry(telemetry);
                    rumble.addTelemetry(telemetry);
                    if (turret != null) turret.addTelemetry(telemetry);
                    loop.addTelemetry(telemetry);
                }
                telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(drivetrain::stop, intake == null ? null : intake::stop,
                    transfer == null ? null : transfer::stop, gate == null ? null : gate::close,
                    turret == null ? null : turret::stop, lights == null ? null : lights::stop,
                    slide == null ? null : slide::stop, () -> DriverRumble.stop(gamepad1));
        }
    }

    /** Rumble is only a driver cue: a failed call must never stop the match OpMode. */
    private static void rumble(Gamepad gamepad, int milliseconds) {
        try { gamepad.rumble(milliseconds); } catch (RuntimeException ignored) { }
    }

    private static void rumbleBlips(Gamepad gamepad, int count) {
        try { gamepad.rumbleBlips(count); } catch (RuntimeException ignored) { }
    }

}
