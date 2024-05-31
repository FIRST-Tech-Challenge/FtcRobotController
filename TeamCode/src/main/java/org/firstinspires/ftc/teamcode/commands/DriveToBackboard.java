package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.Tele;
import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.org.rustlib.drive.Path;
import org.firstinspires.ftc.teamcode.org.rustlib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

public class DriveToBackboard extends Command {

    private final Drive drive;
    private final FlashLights flashLights;
    private final Gamepad gamepad;

    public DriveToBackboard(Drive drive, Lights lights, Gamepad gamepad) {
        this.drive = drive;
        flashLights = new FlashLights(lights, 1000);
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d botPose = drive.odometry.getPose();
        drive.base.setFollowPath(Path.getBuilder().setDefaultRadius(DriveConstants.defaultFollowRadius)
                .addWaypoint(botPose.x, botPose.y).addWaypoint(Tele.backdropPose.toWaypoint()).build());
        flashLights.schedule();
    }

    @Override
    public void execute() {
        drive.base.followPath();
        if (timeSinceInitialized() > 250 && !gamepad.atRest()) {
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return drive.base.finishedFollowing();
    }

    @Override
    public void end(boolean interrupted) {
        flashLights.cancel();
    }
}
