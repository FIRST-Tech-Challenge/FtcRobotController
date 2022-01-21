package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.commandftc.opModes.CommandBasedAuto;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.drive.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.lib.DashboardUtil;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public abstract class BaseAuto extends CommandBasedAuto {
    protected DriveTrainSubsystem driveTrain;
    protected ArmSubsystem armSubsystem;

    @Override
    public void plan() {
        driveTrain = new DriveTrainSubsystem();
        armSubsystem = new ArmSubsystem();
        armSubsystem.setVerticalPosition(0);

        initialize();

        TelemetryPacket packet = new TelemetryPacket();
        DashboardUtil.drawRobot(packet.fieldOverlay(), driveTrain.getPoseEstimate());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    abstract public void initialize();

    protected Command follow(Trajectory trajectory) {
        return new FollowTrajectoryCommand(driveTrain, trajectory);
    }

    protected Command follow(TrajectorySequence trajectory) {
        return new FollowTrajectorySequenceCommand(driveTrain, trajectory);
    }

    protected Command forward(double distance) {
        return new FollowTrajectoryCommand(
                driveTrain,
                driveTrain.trajectoryBuilder(new Pose2d(0, 0, 0)).forward(distance).build()
        );
    }

    protected Command turn(double angle) {
        return new TurnCommand(driveTrain, angle).andThen(new WaitCommand(1));
    }

}
