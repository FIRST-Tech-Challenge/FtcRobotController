package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.drive.Waypoint;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.org.rustlib.rustboard.Rustboard;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Placer;

public class BackdropAlign extends Command {
    private final Drive drive;
    private final Placer placer;
    private double backdropOffset = 0;
    private final double timeout;

    public BackdropAlign(Drive drive, Placer placer, double timeout) {
        this.drive = drive;
        this.placer = placer;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        Pose2d botPose = drive.odometry.getPose();
        backdropOffset = Rustboard.getDoubleValue("place offset", 3.5);
        ;
        drive.base.driveToPosition(new Waypoint(botPose.x - (placer.getDistance() - backdropOffset), botPose.y, 0, Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-90), 0.5));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Rotation2d.getError(Math.toRadians(270), drive.odometry.getPose().rotation.getAngleRadians())) < Math.toRadians(2.5)
                && Math.abs(placer.getDistance() - backdropOffset) < 0.5
                || timeSinceInitialized() > timeout;
    }
}
