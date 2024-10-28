package org.nknsd.robotics.team.autonomous.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;

public class AutoStepMoveNRotate implements NKNAutoStep {
    AutoSkeleton autoSkeleton;
    boolean done = false;
    private double heading;
    private double xDist;
    private double yDist;

    public AutoStepMoveNRotate(double xDist, double yDist, double heading) {
        this.heading = heading;
        this.xDist = xDist;
        this.yDist = yDist;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetPosition(autoSkeleton.targetPositions[0] + xDist, autoSkeleton.targetPositions[1] + yDist);
        autoSkeleton.setTargetRotation(heading);
    }

    @Override
    public void run(Telemetry telemetry) {
        done = autoSkeleton.runToPosition(telemetry);
    }

    @Override
    public boolean isDone(ElapsedTime runtime) {return done;}

    @Override
    public String getName() {
        return "Rotating to " + heading;
    }
}
