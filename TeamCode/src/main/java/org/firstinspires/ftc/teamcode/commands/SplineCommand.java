package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class SplineCommand extends PathCommand {
    public Trajectory trajectory;
    public SplineCommand(DrivebaseSubsystem sub, double x, double y, double endTan, boolean reversed) {
        super(sub);
        trajectory = sub.trajectoryBuilder(sub.getPoseEstimate(), reversed)
                .splineTo(new Vector2d(x, y), Math.toRadians(endTan))
                .build();
    }


    public SplineCommand(DrivebaseSubsystem sub, double x, double y, double endTan){
        this(sub, x, y, endTan, false);
    }

    @Override
    public void init() {
        subsystem.followTrajectoryAsync(trajectory);
    }

}
