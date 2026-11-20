package org.firstinspires.ftc.teamcode;

/**
 * Competition TeleOp's shot, feed, transfer and intake arbitration as one shared coordinator. {@link MecanumTeleOp}
 * drives it from the gamepads (RT aim, RT + LT automatic fire, GP1 A / GP2 RT manual feed, GP2 LT inhibit, bumpers);
 * {@link PlannerRobot} drives it from path-planner steps (volley, aim, spin, shoot, intake, transfer, gate), so a
 * planner auto aims, spins up, waits for readiness and feeds with exactly the code TeleOp uses.
 *
 * <p>Plain class: no hardware of its own, no loops, no sleep(). Per loop, in TeleOp's order:
 * <ol>
 *   <li>{@link #beginLoop} (keeps the firing coordinator in competition continuous mode);</li>
 *   <li>after {@code turret.update()} and any manual yaw: {@link #aim}, then the caller's
 *       {@code turret.applyYawCommand()} (or {@link #aimUnavailable} when there is no turret);</li>
 *   <li>{@link #feed}: gate, transfer and intake from this loop's firing state.</li>
 * </ol>
 */
public final class CompetitionShotPipeline {
    /** Manual / idle flywheel output RPM from START (3000 since 2026-11-19, was 3500); TeleOp GP2 Y/A adjusts it. */
    public static final double FLYWHEEL_IDLE_RPM = 3000.0;
    /** Competition TeleOps and planner autos regulate the flywheel with bang-bang + feedforward fusion. */
    public static final TurretSubsystem.ShooterController FLYWHEEL_CONTROLLER =
            TurretSubsystem.ShooterController.BANG_BANG_FUSION;

    /** One loop's shot requests. TeleOp fills it from the gamepads every loop; planner steps from their state. */
    public static final class Request {
        /** TeleOp RT: automatic aim with ShotCoordinator (Limelight-corrected pose when available). */
        public boolean aim;
        /** Channels automatic aim controls. TeleOp: all three. Planner aim-only / spin-up-only clear some. */
        public boolean controlYaw = true, controlHood = true, controlRpm = true;
        /** TeleOp RT + LT: feed once the turret, hood and flywheel are ready and stable. */
        public boolean automaticFeed;
        /** TeleOp GP1 A / GP2 RT: continuous feed now, readiness bypassed. */
        public boolean manualFeed;
        /** TeleOp GP2 LT: gate closed and transfer stopped; overrides firing. */
        public boolean inhibit;
        /** TeleOp RB (forced forward, past full) / LB (reverse). */
        public boolean intakeForward, intakeReverse;
        /** TeleOp's automatic collection: forward while the queue has space. False pauses it (planner intake OFF). */
        public boolean intakeAutomatic = true;
        /** Scale on the arbitrated intake power (TeleOp 1). */
        public double intakePowerScale = 1.0;
        /** Pause stage-1 loading (planner transfer HOLD; TeleOp false). */
        public boolean holdLoading;
        /** Hold the gate open outside a shot (planner gate OPEN; TeleOp false). Inhibit still closes it. */
        public boolean gateHeldOpen;
        /** Flywheel output RPM when not automatically setting RPM, or when auto aim has no calibrated solution. */
        public double manualRpm = FLYWHEEL_IDLE_RPM;
    }

    private final CompetitionFeedController feedControl = new CompetitionFeedController();
    private final IntakeQueueController intakeControl = new IntakeQueueController();
    private boolean shotReady;
    private double intakePower;

    /** Call first, before the turret update: keeps firing in competition continuous mode. */
    public void beginLoop(double now, FiringCoordinator firing) {
        feedControl.updateSelection(now, firing);
    }

    /**
     * TeleOp's RT pipeline, after {@code turret.update()} and manual yaw: the manual flywheel speed while not
     * automatically setting RPM, the auto-aim trims, {@link ShotCoordinator#update} (aim, readiness and the firing
     * state machine), and the manual-RPM fallback when auto aim has no calibrated solution. The caller writes the
     * yaw servos next with {@code turret.applyYawCommand()}.
     */
    public void aim(double now, TurretSubsystem turret, ShotCoordinator shots, MotionSample sample,
                    AutoAimAdjustments trims, Request request) {
        if (!request.aim || !request.controlRpm) turret.setShooterRpm(request.manualRpm);
        shots.setAutoAimAdjustments(now, trims);
        shots.update(now, sample, request.aim, request.automaticFeed, request.manualFeed, request.inhibit, false,
                request.controlYaw, request.controlHood, request.controlRpm);
        if (request.aim && request.controlRpm && !shots.solution().aimable) turret.setShooterRpm(request.manualRpm);
        shotReady = shots.automaticTrackingReady();
    }

    /** No turret / shot coordinator: manual feed still works (TeleOp's fallback). */
    public void aimUnavailable(double now, FiringCoordinator firing, Request request) {
        firing.update(now, false, request.manualFeed, request.inhibit, false, false, false, false, "Turret unavailable");
        shotReady = false;
    }

    /**
     * Gate, transfer and intake from this loop's firing state (TeleOp's rules): the gate follows the firing
     * coordinator, stage 1 feeds for the whole feed and otherwise loads until full, and automatic collection
     * pauses while shooting and stops on a confirmed full queue.
     *
     * @param readTransferSensors true: {@code transfer.update()} reads the queue sensor now (TeleOp); false: the
     *                            caller already called {@code transfer.readSensors()} this loop (autonomous)
     */
    public void feed(double now, FiringCoordinator firing, GateSubsystem gate, TransferSubsystem transfer,
                     IntakeSubsystem intake, Request request, boolean readTransferSensors) {
        boolean openGate = firing.gateOpen();
        boolean feedStage1 = feedControl.feedStage1(now, firing);
        // Never true in continuous mode; kept so the transfer logic matches the shared coordinator.
        boolean closedGateStage1 = feedControl.closedGateStage1(firing)
                && !request.inhibit && !request.intakeReverse;
        gate.setOpen(openGate || (request.gateHeldOpen && !request.inhibit));
        boolean shootingBusy = request.automaticFeed || request.manualFeed || openGate
                || firing.state() == FiringCoordinator.State.RECOVERY;
        boolean feedBalls = feedControl.feedTransfers(firing, gate.isOpenForFeeding());
        boolean pauseLoading = !closedGateStage1
                && (request.inhibit || shootingBusy || request.intakeReverse || request.holdLoading);
        boolean forceStage1 = closedGateStage1 || (request.intakeForward && !request.intakeReverse);
        if (readTransferSensors) transfer.update(now, feedBalls, pauseLoading, feedStage1, forceStage1);
        else transfer.updateCached(now, feedBalls, pauseLoading, feedStage1, forceStage1);
        intakePower = intakeControl.update(transfer.isFull(), request.intakeForward, request.intakeReverse,
                request.inhibit || shootingBusy || !request.intakeAutomatic);
        intake.setIntakePower(intakePower * request.intakePowerScale);
    }

    /** TeleOp's "shot ready" (lights, rumble): current turret / hood / RPM checks; firing adds the stable interval. */
    public boolean shotReady() { return shotReady; }
    /** Arbitrated intake direction from the last {@link #feed}: 1 forward, -1 reverse, 0 stopped. */
    public double intakePower() { return intakePower; }
}
