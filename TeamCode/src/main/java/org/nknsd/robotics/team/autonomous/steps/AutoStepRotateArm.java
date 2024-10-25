package org.nknsd.robotics.team.autonomous.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;
import org.nknsd.robotics.team.components.RotationHandler;

public class AutoStepRotateArm implements NKNAutoStep {
    private final RotationHandler.RotationPositions rotationPosition;
    AutoSkeleton autoSkeleton;
    boolean done = false;

    public AutoStepRotateArm(RotationHandler.RotationPositions rotationPosition) {
        this.rotationPosition = rotationPosition;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetArmRotation(rotationPosition);
    }

    @Override
    public void run(Telemetry telemetry) {
        done = autoSkeleton.isArmRotationAtTarget();
    }

    @Override
    public boolean isDone(ElapsedTime runtime) {
        return done;
    }

    @Override
    public String getName() {
        return "Rotating to " + rotationPosition.name();
    }
}
