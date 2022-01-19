package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public abstract class BaseAuto extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ArmSubsystem armSubsystem;

    @Override
    public void plan() {
        driveTrain = new DriveTrainSubsystem();
        armSubsystem = new ArmSubsystem();
        armSubsystem.setVerticalPosition(0);

        initialize();
    }

    abstract public void initialize();

    protected Command forward(double distance) {
        return new FollowTrajectoryCommand(
                driveTrain,
                driveTrain.trajectoryBuilder(new Pose2d(0, 0, 0)).forward(distance).build()
        );
    }
}
