package org.firstinspires.ftc.teamcode.commandBased.commands._rr;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoDrivetrainSubsystem;

public class TrajectoryFollowerCommand extends CommandBase {

    private final AutoDrivetrainSubsystem drive;
    private final Trajectory trajectory;

    public TrajectoryFollowerCommand(AutoDrivetrainSubsystem drive, Trajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}
