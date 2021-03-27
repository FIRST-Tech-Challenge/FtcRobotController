package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class PathCommand extends Command {
    public DrivebaseSubsystem subsystem;


    public PathCommand(DrivebaseSubsystem sub){
        addRequirements(sub.dummySubsystem);
        subsystem = sub;
    }


    @Override
    public void execute() {
        subsystem.update();
    }

    @Override
    public boolean isFinished() {
        return !subsystem.isBusy();
    }

    @Override
    public void end(boolean cancel) {
        subsystem.setDriveSignal(new DriveSignal());
    }

    public PathCommand turn(double degrees){
        return path(new TurnCommand(subsystem, degrees));
    }

    public PathCommand spline(double x, double y, double end, boolean reversed){
        return path(new SplineCommand(subsystem, x, y, end, reversed));
    }

    public PathCommand strafe(double x, double y, boolean reversed){
        return path(new StrafeCommand(subsystem, x, y, reversed));
    }

    public PathCommand path(PathCommand c){
        CommandScheduler.getInstance().scheduleAfterOther(this, c);
        return c;
    }
}
