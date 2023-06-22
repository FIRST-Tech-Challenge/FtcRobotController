package org.firstinspires.ftc.teamcode.commandBased.commands._auto;

import org.firstinspires.ftc.teamcode.classes.triggers.TriggerCommandBase;
import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequenceCommand extends TriggerCommandBase {

    private final SampleMecanumDrive drive;
    private final TrajectorySequence traj;

    public FollowTrajectorySequenceCommand(SampleMecanumDrive drive, TrajectorySequence traj) {
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(traj);
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
