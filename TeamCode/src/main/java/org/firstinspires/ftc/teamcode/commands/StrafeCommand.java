package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class StrafeCommand extends PathCommand {
    public double xpos, ypos;
    public boolean reversed;
    public StrafeCommand(DrivebaseSubsystem sub, double x, double y, boolean rev) {
        super(sub);
        xpos = x;
        ypos = y;
        reversed = rev;
    }


    public StrafeCommand(DrivebaseSubsystem sub, double x, double y){
        this(sub, x, y, false);
    }

    @Override
    public void init() {
        subsystem.followTrajectoryAsync(subsystem.trajectoryBuilder(subsystem.getPoseEstimate(), reversed)
                .strafeTo(new Vector2d(xpos,ypos))
                .build());
    }

}
