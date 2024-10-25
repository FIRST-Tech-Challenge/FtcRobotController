package org.nknsd.robotics.team.autonomous.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;

public class AutoStepD implements NKNAutoStep {
    AutoSkeleton autoSkeleton;
    boolean done = false;
    private double dist;

    public AutoStepD(double dist) {
        this.dist = dist;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(autoSkeleton.targetPositions[0], autoSkeleton.targetPositions[1]-dist); // X/Y are weird, sorry future me!
    }

    @Override
    public void run(Telemetry telemetry) {
        done = autoSkeleton.runToPosition(telemetry);
    }

    @Override
    public boolean isDone(ElapsedTime runtime) {return done;}

    @Override
    public String getName() {
        return "Down " + dist;
    }
}
