package org.firstinspires.ftc.teamcode.command;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.BasicDriveTrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This run command make sure the telemetry will be updated after each run command finished, no
 * matter an exception thrown or not from runnable.
 */
public class SounderBotBaseRunCommand<T extends BasicDriveTrain> extends RunCommand {

    private final T driveTrain;

    private boolean finished;

    public SounderBotBaseRunCommand(@Nullable T driveTrain, @NonNull Telemetry telemetry, @NonNull Runnable toRun, Subsystem... requirements) {
        super(() -> {
            try {
                toRun.run();
            } finally {
                telemetry.update();
            }
        }, addDriveTrainToRequirements(driveTrain, requirements));
        this.driveTrain = driveTrain;
        finished = false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (driveTrain != null) {
            driveTrain.stop();
        }
    }

    @Override
    public void execute() {
        super.execute();
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    /**
     * add drive train to requirements
     */
    private static <U extends Subsystem> Subsystem[] addDriveTrainToRequirements(@Nullable U driveTrain, Subsystem... requirements) {
        if (driveTrain == null) {
            return requirements;
        }
        List<Subsystem> result = new ArrayList<>();
        if (requirements != null) {
            result.addAll(Arrays.asList(requirements));
        }

        result.add(driveTrain);
        return result.toArray(new Subsystem[0]);
    }

    /**
     * Create an empty command that will only update the telemetry
     */
    public static SounderBotBaseRunCommand<?> createTelemetryEnabledOnlyInstance(Telemetry telemetry) {
        return new SounderBotBaseRunCommand<>(null, telemetry, () -> {});
    }
}
