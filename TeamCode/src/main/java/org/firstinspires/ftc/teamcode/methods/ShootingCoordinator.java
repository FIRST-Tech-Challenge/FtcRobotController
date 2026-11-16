package org.firstinspires.ftc.teamcode.methods;

import java.util.function.DoubleSupplier;

/** Competition shooter: readiness, 100 ms transfer ramp, stored-batch feed (continuous or paced). */
public final class ShootingCoordinator {
    /**
     * Feed mode used by MainTeleOp and every auto. CONTINUOUS is the original behavior.
     * Switch to PACED only after the Volley Consistency Test shows it scores better.
     */
    public static final SynchronizedFeed.Mode COMPETITION_FEED_MODE = SynchronizedFeed.Mode.CONTINUOUS;

    private final Launcher launcher;
    private final Transfer transfer;
    private final Intake intake;
    private final SynchronizedFeed feed = new SynchronizedFeed();
    private final DoubleSupplier clock;

    public ShootingCoordinator(Launcher launcher, Transfer transfer, Intake intake) {
        this(launcher, transfer, intake, () -> System.nanoTime() / 1e9);
    }
    ShootingCoordinator(Launcher launcher, Transfer transfer, Intake intake, DoubleSupplier clock) {
        this.launcher = launcher; this.transfer = transfer; this.intake = intake; this.clock = clock;
        feed.setMode(COMPETITION_FEED_MODE);
    }

    /** Reverse has priority; releasing it retains the launcher's prior on/off state. */
    public void update(boolean shoot, boolean collect, boolean reverse) {
        update(shoot, collect, reverse, true);
    }

    /** waitForFlywheel=false gives the TeleOp operator unconditional feed control. */
    public void update(boolean shoot, boolean collect, boolean reverse, boolean waitForFlywheel) {
        if (reverse) {
            feed.cancel();
            launcher.outtake(); transfer.outtake(); intake.outtake();
            return;
        }
        launcher.stopOuttake(); transfer.stopOuttake();
        launcher.update();
        feed.update(clock.getAsDouble(), shoot, launcher.isFlywheelsOn(),
                launcher.getTargetVelocity(), launcher.getMeasuredVelocity(), transfer.getMeasuredVelocity(), waitForFlywheel);
        if (feed.transferTarget() > 0) transfer.setTargetVelocity(feed.transferTarget());
        else transfer.stopAll();
        transfer.update();
        // While a shoot request is waiting for readiness (or a paced shot is recovering),
        // collection cannot push a stored ball into the flywheel.
        if (feed.intakeEnabled() || (!shoot && collect)) intake.intake();
        else intake.stop();
        intake.update();
    }

    /** Experiment settings; allowed only while no shot request is active. */
    public void setFeedMode(SynchronizedFeed.Mode mode) { feed.setMode(mode); }
    public void setTransferSetpoint(double ticksPerSecond) { feed.setTransferSetpoint(ticksPerSecond); }
    public void setPacedRecovery(double tolerance, double dwellSeconds, double maxSeconds) {
        feed.setPacedRecovery(tolerance, dwellSeconds, maxSeconds);
    }

    public SynchronizedFeed.State state() { return feed.state(); }
    public SynchronizedFeed.Mode feedMode() { return feed.mode(); }
    public boolean isFeeding() { return feed.intakeEnabled(); }
    public int shotCount() { return feed.shotCount(); }
    public boolean shotDetected() { return feed.shotDetected(); }
    public double lastRecoverSeconds() { return feed.lastRecoverSeconds(); }
    public int recoverTimeouts() { return feed.recoverTimeouts(); }
    public double feedSeconds() { return feed.feedSeconds(); }
    public void stopFeed() { feed.cancel(); transfer.stopAll(); intake.stop(); }
    public void stopAll() { stopFeed(); launcher.stopAll(); }
}
