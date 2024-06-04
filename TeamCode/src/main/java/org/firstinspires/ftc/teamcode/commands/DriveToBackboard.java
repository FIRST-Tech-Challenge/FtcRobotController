package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.opmodes.tele.Tele;
import org.firstinspires.ftc.teamcode.subsystems.Lights;
import org.rustlib.commandsystem.Command;
import org.rustlib.drive.DriveSubsystem;
import org.rustlib.drive.Path;
import org.rustlib.geometry.Pose2d;

public class DriveToBackboard extends Command {

    private final DriveSubsystem drive;
    private final FlashLights flashLights;
    private final Gamepad gamepad;

    public DriveToBackboard(DriveSubsystem drive, Lights lights, Gamepad gamepad) {
        this.drive = drive;
        flashLights = new FlashLights(lights, 1000);
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d botPose = drive.getOdometry().getPose();
        drive.getBase().setFollowPath(Path.getBuilder().setDefaultRadius(DriveConstants.defaultFollowRadius)
                .addWaypoint(botPose.x, botPose.y).addWaypoint(Tele.backdropPose.toWaypoint()).build());
        flashLights.schedule();
    }

    @Override
    public void execute() {
        drive.getBase().followPath();
        if (timeSinceInitialized() > 250 && !gamepad.atRest()) {
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return drive.getBase().finishedFollowing();
    }

    @Override
    public void end(boolean interrupted) {
        flashLights.cancel();
    }
}
