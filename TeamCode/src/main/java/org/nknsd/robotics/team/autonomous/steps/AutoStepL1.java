package org.nknsd.robotics.team.autonomous.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;

public class AutoStepL1 implements NKNAutoStep {
    AutoSkeleton autoSkeleton;
    boolean done = false;

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;
    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(0, -24); // Positions are weird, sorry future me!
    }

    @Override
    public void run(Telemetry telemetry) {
        done = autoSkeleton.runToPosition(telemetry);
    }

    @Override
    public boolean isDone(ElapsedTime runtime) {
        return done;
    }

    @Override
    public String getName() {
        return "Left 1";
    }
}
