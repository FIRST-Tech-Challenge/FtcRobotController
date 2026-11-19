package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

/** Latched desired outputs, independent of Ivy's temporary command locks. No hardware reads. */
public final class AutonomousOutputCoordinator {
    public enum Mode { MANUAL, AUTO, OFF }
    private final Map<String, Long> generations = new HashMap<>();
    private Mode yaw = Mode.MANUAL, hood = Mode.MANUAL, flywheel = Mode.OFF;
    private boolean loading, automaticFeed, manualFeed, gateOnly, inhibited;
    private double intakePower, rpm;
    private final StaggeredFeedTiming stage1Timing = new StaggeredFeedTiming();
    private AutonomousShotPresets.Preset preset;

    /** A timed command may clean up only channels it still owns. */
    public Map<String, Long> claim(String... channels) {
        Map<String, Long> lease = new HashMap<>();
        for (String channel : channels) {
            long next = generations.getOrDefault(channel, 0L) + 1;
            generations.put(channel, next); lease.put(channel, next);
        }
        return lease;
    }
    public boolean owns(Map<String, Long> lease, String channel) {
        return lease != null && lease.containsKey(channel) && lease.get(channel).equals(generations.get(channel));
    }
    public void intake(double power) { intakePower = power; }
    /** Forward collection stops on a confirmed full queue; deliberate reverse is still allowed. */
    public double intakePower(boolean queueFull) {
        return queueFull && intakePower > 0 ? IntakeQueueController.FULL_QUEUE_INTAKE_POWER : intakePower;
    }
    public void loading(boolean enabled) { loading = enabled; }
    public boolean loading() { return loading; }
    public void aim(boolean spin) {
        preset = null; yaw = hood = Mode.AUTO; flywheel = spin ? Mode.AUTO : Mode.OFF;
        clearFeed();
    }
    public void aimOff() { yaw = hood = Mode.MANUAL; flywheel = Mode.OFF; rpm = 0; preset = null; clearFeed(); }
    public void manualYaw() { yaw = Mode.MANUAL; preset = null; automaticFeed = false; }
    public void manualHood() { hood = Mode.MANUAL; preset = null; automaticFeed = false; }
    public void rpm(double value) { flywheel = value == 0 ? Mode.OFF : Mode.MANUAL; rpm = value; preset = null; automaticFeed = false; }
    public void prepare(AutonomousShotPresets.Preset value) {
        preset = value; yaw = hood = Mode.MANUAL; flywheel = Mode.MANUAL; rpm = value.outputRpm; clearFeed();
    }
    public void requestAutomaticFeed() { automaticFeed = true; manualFeed = gateOnly = false; }
    public void requestManualFeed() { manualFeed = true; automaticFeed = gateOnly = false; }
    public void gateOnly() { clearFeed(); gateOnly = true; loading = false; }
    public void clearFeed() { automaticFeed = manualFeed = gateOnly = false; }
    public void inhibit(boolean value) { inhibited = value; clearFeed(); }
    public void stopAll() {
        intakePower = 0; loading = false; aimOff();
        claim("drive", "intake", "transfer", "gate", "turretYaw", "hood", "flywheel", "depollinator", "pollenCamera", "limelight");
    }
    public Mode yawMode() { return yaw; }
    public Mode hoodMode() { return hood; }
    public Mode flywheelMode() { return flywheel; }
    public boolean aiming() { return yaw == Mode.AUTO || hood == Mode.AUTO || flywheel == Mode.AUTO; }
    public boolean automaticFeed() { return automaticFeed && !inhibited &&
            (preset != null || (yaw == Mode.AUTO && hood == Mode.AUTO && flywheel == Mode.AUTO)); }
    public boolean manualFeed() { return manualFeed && !inhibited; }
    public boolean feedRequested() { return automaticFeed || manualFeed; }
    public boolean inhibited() { return inhibited; }
    public boolean gateOnlyRequested() { return gateOnly && !inhibited; }

    /** Apply resolved mechanism requests once, after this loop's scheduler execution. */
    public void apply(double now, MotionSample sample, TurretSubsystem turret, ShotCoordinator shots,
                      FiringCoordinator firing, GateSubsystem gate, TransferSubsystem transfer, IntakeSubsystem intake) {
        if (shots != null) {
            if (preset != null) shots.updatePreset(now, sample, preset, automaticFeed(), manualFeed(), inhibited);
            else shots.update(now, sample, aiming(), automaticFeed(), manualFeed(), inhibited,
                    flywheel == Mode.OFF, yaw == Mode.AUTO, hood == Mode.AUTO, flywheel == Mode.AUTO);
            if (flywheel == Mode.MANUAL) turret.setShooterRpm(rpm);
        } else firing.update(now, false, manualFeed(), inhibited, false, false, false, false, "No shooting hardware");
        // Same-loop servo write of the yaw target resolved above.
        if (turret != null) turret.applyYawCommand(now);
        gate.setOpen(!inhibited && (gateOnlyRequested() || firing.gateOpen()));
        boolean feeding = firing.feeding() && gate.isOpenForFeeding() && !inhibited;
        // Same stage-1 policy as TeleOp: stage 1 starts with stage 2 on every feed, and
        // stage 1 alone keeps running with the gate closed between timed feeds.
        boolean feedStage1 = stage1Timing.feedStage1(now, firing);
        boolean closedGateStage1 = !inhibited && firing.closedGateStage1();
        transfer.updateCached(now, feeding,
                !closedGateStage1 && (!loading || feedRequested() || gateOnlyRequested()),
                feedStage1, closedGateStage1);
        intake.setIntakePower(intakePower(transfer.isFull()));
        transfer.setIntakeRunning(intake.getCommandedPower() > 0); // Stage 1 stall guard only while intaking.
    }
}
