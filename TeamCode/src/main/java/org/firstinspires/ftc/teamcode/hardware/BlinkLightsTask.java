package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.jetbrains.annotations.NotNull;

import dev.aether.collaborative_multitasking.Scheduler;
import dev.aether.collaborative_multitasking.TaskTemplate;

public class BlinkLightsTask extends TaskTemplate {
    private final Hardware hardware;
    private final double duration;
    private final double period;
    private final double color1;
    private final double color2;
    private final double colorFinal;
    private final ElapsedTime elapsed = new ElapsedTime();

    private boolean left = false;
    private boolean right = false;

    private double lastIter = 0;


    public BlinkLightsTask(@NotNull Scheduler scheduler, Hardware hardware, double duration, boolean alternate, double freq, double color1, double color2, double colorFinal) {
        super(scheduler);
        this.hardware = hardware;
        this.duration = duration;
        if (alternate) right = true;
        this.period = 1.0 / freq;
        this.color1 = color1;
        this.color2 = color2;
        this.colorFinal = colorFinal;
    }

    private void update() {
        double leftCol = left ? color1 : color2;
        double rightCol = right ? color1 : color2;
        hardware.lightLeft.setPosition(leftCol);
        hardware.lightRight.setPosition(rightCol);
    }

    @Override
    public void invokeOnStart() {
        elapsed.reset();
        update();
    }

    @Override
    public void invokeOnTick() {
        if (elapsed.time() - lastIter > period) {
            left = !left;
            right = !right;
            lastIter = elapsed.time();
            update();
        }
    }

    @Override
    public boolean invokeIsCompleted() {
        return elapsed.time() >= duration;
    }

    @Override
    public void invokeOnFinish() {
        hardware.lightLeft.setPosition(colorFinal);
        hardware.lightRight.setPosition(colorFinal);
    }
}
