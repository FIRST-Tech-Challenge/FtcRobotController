package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class SplineCommand extends PathCommand {
    public double xpos, ypos, endTanganet;
    public boolean reversed;
    public SplineCommand(DrivebaseSubsystem sub, double x, double y, double endTan, boolean rev) {
        super(sub);
        xpos = x;
        ypos = y;
        endTanganet = endTan;
        reversed = rev;
    }


    public SplineCommand(DrivebaseSubsystem sub, double x, double y, double endTan){
        this(sub, x, y, endTan, false);
    }

    @Override
    public void init() {
        subsystem.followTrajectoryAsync(subsystem.trajectoryBuilder(subsystem.getPoseEstimate(), reversed)
                .splineTo(new Vector2d(xpos,ypos), Math.toRadians(endTanganet))
                .build());
    }

}
