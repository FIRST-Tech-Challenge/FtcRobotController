package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class StrafeCommand extends PathCommand {
    public Trajectory trajectory;
    public StrafeCommand(DrivebaseSubsystem sub, double x, double y, boolean reversed) {
        super(sub);
        trajectory = sub.trajectoryBuilder(sub.getPoseEstimate(), reversed)
                .strafeTo(new Vector2d(x,y))
                .build();
    }


    public StrafeCommand(DrivebaseSubsystem sub, double x, double y){
        this(sub, x, y, false);
    }

    @Override
    public void init() {
        subsystem.followTrajectoryAsync(trajectory);
    }

}
