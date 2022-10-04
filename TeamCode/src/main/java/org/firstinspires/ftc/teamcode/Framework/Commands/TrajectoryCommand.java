package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class TrajectoryCommand extends CommandBase {

    private SampleMecanumDrive drive;
    private Trajectory trajectory;


    public TrajectoryCommand(SampleMecanumDrive drive, Trajectory trajectory){
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void execute(){
        drive.followTrajectory(trajectory);
    }

    @Override
    public boolean isFinished(){
        return drive.isBusy();
    }
}
