package org.nknsd.robotics.team.autonomous.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;

public class AutoStepAdjustTarget implements NKNAutoStep {
    AutoSkeleton autoSkeleton;
    private double xDist;
    private double yDist;

    public AutoStepAdjustTarget(double xDist, double yDist) {
        this.xDist = xDist;
        this.yDist = yDist;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(autoSkeleton.targetPositions[0] + xDist, autoSkeleton.targetPositions[1] + yDist); // X/Y are weird, sorry future me!
    }

    @Override
    public void run(Telemetry telemetry) {}

    @Override
    public boolean isDone(ElapsedTime runtime) {return true;}

    @Override
    public String getName() {
        return "Adjusting target";
    }
}
