package org.firstinspires.ftc.teamcode.Framework.Commands;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class TrajectoryCommand extends CommandBase {

    private SampleMecanumDrive drive;
    private Trajectory trajectory;


    public TrajectoryCommand(AutoDrive drivetrain, Trajectory trajectory){
        this.drive = drivetrain.getDrive();
        this.trajectory = trajectory;
        addRequirements(drivetrain);
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
