package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Placer;
import org.rustlib.commandsystem.Command;
import org.rustlib.drive.DriveSubsystem;
import org.rustlib.drive.Waypoint;
import org.rustlib.geometry.Pose2d;
import org.rustlib.geometry.Rotation2d;
import org.rustlib.rustboard.RustboardServer;

public class BackdropAlign extends Command {
    private final DriveSubsystem drive;
    private final Placer placer;
    private final double timeout;
    private double backdropOffset = 0;

    public BackdropAlign(DriveSubsystem drive, Placer placer, double timeout) {
        this.drive = drive;
        this.placer = placer;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        Pose2d botPose = drive.getOdometry().getPose();
        backdropOffset = RustboardServer.getDoubleValue("place offset", 3.5);
        drive.getBase().driveToPosition(new Waypoint(botPose.x - (placer.getDistance() - backdropOffset), botPose.y, 0, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90), 0.5));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Rotation2d.getError(Math.toRadians(270), drive.getOdometry().getPose().rotation.getAngleRadians())) < Math.toRadians(2.5)
                && Math.abs(placer.getDistance() - backdropOffset) < 0.5
                || timeSinceInitialized() > timeout;
    }
}
