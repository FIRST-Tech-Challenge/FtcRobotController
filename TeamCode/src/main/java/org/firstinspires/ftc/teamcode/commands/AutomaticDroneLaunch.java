package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.drive.Waypoint;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class AutomaticDroneLaunch extends Command {
    private final Drive drive;
    private final Command shootDrone;
    private final Gamepad gamepad;
    private Waypoint target;

    public AutomaticDroneLaunch(Drive drive, Command shootDrone, Gamepad gamepad) {
        this.drive = drive;
        this.shootDrone = shootDrone;
        this.gamepad = gamepad;
    }

    @Override
    public void initialize() {
        target = new Waypoint(30, drive.odometry.getPose().y, 0, null, new Rotation2d());
    }

    @Override
    public void execute() {
        target = new Waypoint(30, drive.odometry.getPose().y, 0, null, new Rotation2d());
        drive.base.driveToPosition(target);
    }

    @Override
    public boolean isFinished() {
        return drive.base.atWaypoint(target, 2, Math.toRadians(5))
                || (timeSinceInitialized() > 250 && !gamepad.atRest());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) shootDrone.schedule();
    }
}
