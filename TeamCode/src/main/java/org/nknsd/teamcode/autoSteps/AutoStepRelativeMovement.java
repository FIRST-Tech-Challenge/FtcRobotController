package org.nknsd.teamcode.autoSteps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.frameworks.NKNAutoStep;
import org.nknsd.teamcode.programs.autos.AutoSkeleton;

import java.util.concurrent.TimeUnit;

public class AutoStepRelativeMovement implements NKNAutoStep {
    AutoSkeleton autoSkeleton;
    boolean done = false;
    private final double turning;
    private final double x;
    private final double y;
    private final long duration;
    private long startTime;

    public AutoStepRelativeMovement(double x, double y, double turning, long duration) {
        this.turning = turning;
        this.x = x;
        this.y = y;
        this.duration = duration;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.runMotorsDirectly(y, x, turning);
        startTime = runtime.now(TimeUnit.MILLISECONDS);
    }

    @Override
    public void run(Telemetry telemetry, ElapsedTime runtime) {}

    @Override
    public boolean isDone(ElapsedTime runtime) {
        if (runtime.now(TimeUnit.MILLISECONDS) - startTime > duration) {
            autoSkeleton.freeze();
            return true;
        }
        return false;
    }

    @Override
    public String getName() {
        return "Rotating to " + turning;
    }
}
