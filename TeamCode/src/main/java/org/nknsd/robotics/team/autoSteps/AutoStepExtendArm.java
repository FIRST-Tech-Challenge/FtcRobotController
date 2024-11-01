package org.nknsd.robotics.team.autoSteps;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNAutoStep;
import org.nknsd.robotics.team.autonomous.AutoSkeleton;
import org.nknsd.robotics.team.components.ExtensionHandler;

import java.util.concurrent.TimeUnit;

public class AutoStepExtendArm implements NKNAutoStep {
    private final ExtensionHandler.ExtensionPositions extensionPosition;
    AutoSkeleton autoSkeleton;

    public AutoStepExtendArm(ExtensionHandler.ExtensionPositions extensionPosition) {
        this.extensionPosition = extensionPosition;
    }

    @Override
    public void link(AutoSkeleton autoSkeleton) {
        this.autoSkeleton = autoSkeleton;

    }

    public void begin(ElapsedTime runtime, Telemetry telemetry) {
        autoSkeleton.setTargetArmExtension(extensionPosition);
    }

    @Override
    public void run(Telemetry telemetry) {}

    @Override
    public boolean isDone(ElapsedTime runtime) {
        return autoSkeleton.isExtensionDone();
    }

    @Override
    public String getName() {
        return "Extending to " + extensionPosition.name();
    }
}
