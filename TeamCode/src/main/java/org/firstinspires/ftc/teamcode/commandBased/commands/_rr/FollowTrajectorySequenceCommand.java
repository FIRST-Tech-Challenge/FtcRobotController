package org.firstinspires.ftc.teamcode.commandBased.commands._rr;

import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.AutoDrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequenceCommand extends TriggerCommandBase {

    private final AutoDrivetrainSubsystem drive;
    private final TrajectorySequence traj;

    public FollowTrajectorySequenceCommand(AutoDrivetrainSubsystem drive, TrajectorySequence traj) {
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(traj);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public boolean isTriggered() {
        return true;
    }

}
