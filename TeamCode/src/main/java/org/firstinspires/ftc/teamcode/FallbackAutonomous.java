package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Layer 3 fallback autonomous for when no tuned Pedro route exists: optionally shoot the preloads from
 * the start spot with fixed manual settings, then drive a short distance out of the start area using
 * Pinpoint. The logic lives in {@link FallbackAutoController}; this class only wires hardware.
 *
 * <p>Independent of Pedro, Zenith and shot calibration. Hardware: drivetrain, Pinpoint, gate, transfer and
 * (optional) turret/flywheel. An unavailable turret only disables the preload shot.
 *
 * <p>INIT controls (Gamepad 1): D-pad Up / Down = start delay +/- 1 s (0..15 s); X = toggle preload shot.
 * Keep the robot still during INIT (Pinpoint IMU calibration and turret homing).
 */
public abstract class FallbackAutonomous extends LinearOpMode {

    protected abstract TurretSubsystem.Alliance getAlliance();
    /**
     * Turret heading that aims at the goal from the start spot, in the frame the Driver Hub shows as
     * "Turret | HOMED | heading": front=0, right/home=90, back=180, left=270 deg.
     */
    protected abstract double preloadTurretHeadingDegrees();
    /** Leave target relative to the start pose: + forward, inches. */
    protected abstract double leaveForwardInches();
    /** Leave target relative to the start pose: + left, inches. */
    protected abstract double leaveLeftInches();

    // Preload shot from the start spot. STARTING VALUES, UNMEASURED: TeleOp's starting manual hood (home) and
    // its former 3500 starting RPM (kept here when TeleOp's idle dropped to 3000 on 2026-11-19).
    // Measure a hitting hood / RPM at the start spot, then set MEASURED true.
    public static final double PRELOAD_HOOD_DEG = TurretSubsystem.PITCH_HOME_DEG;
    public static final double PRELOAD_FLYWHEEL_RPM = 3500.0;
    /** Default for the INIT shot toggle. Until measured, the shot starts OFF and GP1 X can enable it. */
    public static final boolean PRELOAD_SHOT_MEASURED = false;

    @Override
    public void runOpMode() {
        // This routine has no field pose. An older autonomous pose must never reach the next TeleOp.
        AutoPoseHandoff.clear();
        RobotLoop loop = new RobotLoop(hardwareMap);
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        GateSubsystem gate = null;
        TransferSubsystem transfer = null;
        TurretSubsystem turret = null;
        try {
            PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
            gate = new GateSubsystem(hardwareMap, CompetitionFeedController.GATE_TO_FEED_DELAY_SECONDS);
            // Turret encoder reads intake_motor quad port 1; TurretSubsystem never reconfigures that channel.
            transfer = new TransferSubsystem(hardwareMap);
            String turretError = null;
            try {
                turret = new TurretSubsystem(hardwareMap, false); // Starts homing; no camera needed.
                turret.setEncoderCorrectionEnabled(true);
            } catch (Exception e) {
                turretError = e.getMessage();
            }

            double startDelay = 0;
            boolean shoot = PRELOAD_SHOT_MEASURED && turret != null;
            boolean previousUp = gamepad1.dpad_up, previousDown = gamepad1.dpad_down, previousX = gamepad1.x;
            while (opModeInInit()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                odometry.update();
                if (turret != null) {
                    turret.update(now);
                    turret.applyYawCommand(now);
                }
                int steps = (gamepad1.dpad_up && !previousUp ? 1 : 0) - (gamepad1.dpad_down && !previousDown ? 1 : 0);
                startDelay = FallbackAutoController.stepDelay(startDelay, steps);
                if (gamepad1.x && !previousX && turret != null) shoot = !shoot;
                previousUp = gamepad1.dpad_up; previousDown = gamepad1.dpad_down; previousX = gamepad1.x;

                telemetry.addData("Fallback auto", "%s | no Pedro, no field pose", getAlliance());
                telemetry.addData("Start delay (GP1 D-pad Up/Down)", "%.0f s", startDelay);
                if (turret == null) telemetry.addData("Preload shot", "UNAVAILABLE: turret %s", turretError);
                else telemetry.addData("Preload shot (GP1 X)", "%s | heading %.1f deg, hood %.1f deg, %.0f RPM%s",
                        shoot ? "ON" : "OFF", preloadTurretHeadingDegrees(), PRELOAD_HOOD_DEG, PRELOAD_FLYWHEEL_RPM,
                        PRELOAD_SHOT_MEASURED ? "" : " | UNMEASURED");
                telemetry.addData("Leave (in)", "forward %.1f | left %.1f", leaveForwardInches(), leaveLeftInches());
                odometry.addDriverTelemetry(telemetry);
                if (turret != null) turret.addDriverTelemetry(telemetry);
                telemetry.addLine("Keep robot still: Pinpoint IMU calibration and turret homing.");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;

            FallbackAutoController auto = new FallbackAutoController(startDelay, shoot,
                    leaveForwardInches(), leaveLeftInches());
            CompetitionFeedController feedControl = new CompetitionFeedController(); // Continuous feed.
            FiringCoordinator firing = CompetitionFeedController.firing();
            auto.start(System.nanoTime() * 1e-9);

            while (opModeIsActive()) {
                loop.begin();
                double now = System.nanoTime() * 1e-9;
                odometry.update();
                boolean homed = false, shooterReady = false;
                if (turret != null) {
                    turret.update(now);
                    homed = turret.isHomed();
                    shooterReady = homed && turret.isShooterAtSpeed() && turret.isHoodTracking()
                            && Math.abs(turret.getTargetYawDegrees() - turret.getEncoderAngleDegrees())
                            <= TurretSubsystem.SHOT_READY_YAW_TOLERANCE_DEG;
                }
                auto.update(now, odometry.getSample(), homed, shooterReady);

                drivetrain.driveRobotCentric(auto.axial(), auto.lateral(), auto.yaw());
                if (turret != null) {
                    if (auto.flywheelOn()) {
                        // Same heading-to-yaw conversion and dead-zone handling as auto aim.
                        if (homed) turret.setTargetYawDegrees(turret.reachableYawTarget(
                                TurretControl.robotHeadingToYawDegrees(preloadTurretHeadingDegrees())));
                        turret.setPitchAngleDegrees(PRELOAD_HOOD_DEG);
                        turret.setShooterRpm(PRELOAD_FLYWHEEL_RPM);
                    } else {
                        turret.stopShooter();
                    }
                    turret.applyYawCommand(now);
                }

                // Same manual-feed path as TeleOp's GP1 A: gate, then the transfer (both motors).
                feedControl.updateSelection(now, firing);
                firing.update(now, false, auto.feedRequested(), false, false, false, false, false,
                        "Fallback autonomous preload feed");
                boolean feedStage1 = feedControl.feedStage1(now, firing);
                gate.setOpen(firing.gateOpen());
                boolean shootingBusy = auto.feedRequested() || firing.gateOpen()
                        || firing.state() == FiringCoordinator.State.RECOVERY;
                // Stage 1 seats the preloads while spinning up, like TeleOp; it rests once the shot is over.
                boolean seatingPreloads = auto.shootPreloads() && (auto.phase() == FallbackAutoController.Phase.START_DELAY
                        || auto.phase() == FallbackAutoController.Phase.SPIN_UP);
                transfer.update(now, feedControl.feedTransfers(firing, gate.isOpenForFeeding()),
                        shootingBusy || !seatingPreloads, feedStage1, false);

                telemetry.addData("Fallback auto", "%s | %s | %.1f s", getAlliance(), auto.phase(), auto.elapsedSeconds(now));
                telemetry.addData("Status", auto.status());
                if (auto.shootPreloads()) telemetry.addData("Preload shot", "%s%s", firing.state(),
                        auto.fedBeforeReady() ? " | fed before READY" : "");
                if (Double.isFinite(auto.remainingInches()))
                    telemetry.addData("Leave remaining (in)", "%.1f", auto.remainingInches());
                odometry.addDriverTelemetry(telemetry);
                if (turret != null) turret.addDriverTelemetry(telemetry);
                else telemetry.addData("Turret", "UNAVAILABLE: %s", turretError);
                transfer.addDriverTelemetry(telemetry);
                loop.addRateTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            RobotShutdown.stopAll(drivetrain::stop, transfer == null ? null : transfer::stop,
                    gate == null ? null : gate::close, turret == null ? null : turret::stop);
        }
    }
}
