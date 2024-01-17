package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceCommand extends CommandBase {

    private SampleMecanumDrive drive;
    private TrajectorySequence trajectory;

    public TrajectorySequenceCommand(SampleMecanumDrive drive, TrajectorySequence trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void initialize(){
        drive.followTrajectorySequenceAsync(trajectory);
    }

    @Override
    public void execute(){
        drive.update();
    }

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }
}