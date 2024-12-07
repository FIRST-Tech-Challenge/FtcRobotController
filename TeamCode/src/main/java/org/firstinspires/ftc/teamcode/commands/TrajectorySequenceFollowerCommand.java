package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceFollowerCommand extends CommandBase {
    private final SampleMecanumDrive drive;
    private final TrajectorySequence trajectory;

    public TrajectoryFollowerCommand(SampleMecanumDrive drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void execute() {
        drive.followTrajectorySequenceAsync(trajectory);
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            trajectory.
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}