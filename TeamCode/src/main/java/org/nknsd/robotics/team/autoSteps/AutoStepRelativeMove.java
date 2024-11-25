package org.nknsd.robotics.team.autoSteps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;

import java.util.concurrent.TimeUnit;

public class AutoStepRelativeMove implements NKNAutoStep {
    AutoSkeleton autoSkeleton;
    private final long time;
    private long startTime;
    private final double x; private final double y;

    public AutoStepRelativeMove(double x, double y, long time) {
        this.x = x;
        this.y = y;
        this.time = time;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;
    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.relativeRun(x, y);
        startTime = runtime.now(TimeUnit.MILLISECONDS);
    }

    @Override
    public void run(Telemetry telemetry, ElapsedTime runtime) {}

    @Override
    public boolean isDone(ElapsedTime runtime) {
        if (runtime.now(TimeUnit.MILLISECONDS) - startTime > time) {
            autoSkeleton.relativeRun(0, 0);
            return true;
        }
        return false;
    }

    @Override
    public String getName() {
        return "Moving without flow sensor";
    }
}
