package org.firstinspires.ftc.teamcode.hardware;

import static androidx.core.math.MathUtils.clamp;

import org.firstinspires.ftc.teamcode.Hardware;
import org.jetbrains.annotations.NotNull;

import java.util.Set;

import dev.aether.collaborative_multitasking.ITask;
import dev.aether.collaborative_multitasking.OneShot;
import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.SharedResource;
import dev.aether.collaborative_multitasking.TaskTemplate;

public class HClawProxy extends TaskTemplate {
    private static final Set<SharedResource> requires = Set.of(Hardware.Locks.HSlideClaw);
    private static int INSTANCE_COUNT = 0;
    public final SharedResource CONTROL_CLAW = new SharedResource("HClawProxy_Claw" + (++INSTANCE_COUNT));
    public final SharedResource CONTROL_FLIP = new SharedResource("HClawProxy_Flip" + (++INSTANCE_COUNT));
    private final Hardware hardware;
    private final Scheduler scheduler = getScheduler();
    private double flipPosition = Hardware.FLIP_UP;
    private double clawPosition = Hardware.FRONT_OPEN;

    public HClawProxy(@NotNull Scheduler scheduler, Hardware hardware) {
        super(scheduler);
        this.hardware = hardware;
    }

    public double getFlipPosition() {
        return flipPosition;
    }

    public double getClawPosition() {
        return clawPosition;
    }

    @Override
    @NotNull
    public Set<SharedResource> requirements() {
        return requires;
    }

    private void update() {
        hardware.clawFlip.setPosition(flipPosition);
        hardware.clawFront.setPosition(clawPosition);
    }

    @Override
    public void invokeOnStart() {
        update();
    }

    public void setFlip(double newPos) {
        flipPosition = clamp(newPos, 0.0, 1.0);
        update();
    }

    public void setClaw(double newPos) {
        clawPosition = clamp(newPos, 0.0, 1.0);
        update();
    }

    public ITask aSetFlip(double newPos) {
        return new OneShot(scheduler, () -> setFlip(newPos));
    }

    public ITask aSetClaw(double newPos) {
        return new OneShot(scheduler, () -> setClaw(newPos));
    }

    public void setFlipClaw(double flip, double claw) {
        flipPosition = clamp(flip, 0.0, 1.0);
        clawPosition = clamp(claw, 0.0, 1.0);
        update();
    }

    public ITask aSetFlipClaw(double flip, double claw) {
        return new OneShot(scheduler, () -> setFlipClaw(flip, claw));
    }

    @Override
    public boolean getDaemon() {
        return true;
    }
}
