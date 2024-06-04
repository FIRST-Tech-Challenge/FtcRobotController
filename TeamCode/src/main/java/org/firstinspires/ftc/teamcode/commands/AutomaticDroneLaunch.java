package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.rustlib.commandsystem.Command;
import org.rustlib.drive.DriveSubsystem;
import org.rustlib.drive.Waypoint;
import org.rustlib.geometry.Rotation2d;

public class AutomaticDroneLaunch extends Command {
    private final DriveSubsystem drive;
    private final Command shootDrone;
    private final Gamepad gamepad;
    private Waypoint target;

    public AutomaticDroneLaunch(DriveSubsystem drive, Command shootDrone, Gamepad gamepad) {
        this.drive = drive;
        this.shootDrone = shootDrone;
        this.gamepad = gamepad;
    }

    @Override
    public void initialize() {
        target = new Waypoint(30, drive.getOdometry().getPose().y, 0, null, new Rotation2d());
    }

    @Override
    public void execute() {
        target = new Waypoint(30, drive.getOdometry().getPose().y, 0, null, new Rotation2d());
        drive.getBase().driveToPosition(target);
    }

    @Override
    public boolean isFinished() {
        return drive.getBase().atWaypoint(target, 2, Math.toRadians(5))
                || (timeSinceInitialized() > 250 && !gamepad.atRest());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) shootDrone.schedule();
    }
}
