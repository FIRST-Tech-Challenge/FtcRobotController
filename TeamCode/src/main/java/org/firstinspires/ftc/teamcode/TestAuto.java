package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.lib.tragectory.TrajectorySequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

@Autonomous
public class TestAuto extends BaseAuto {

    @Override
    public void initialize() {
        driveTrain.setPoseEstimate(new Pose2d(-1, -1.61, Math.toRadians(90)));

        telemetry.addData("heading", () -> Math.toDegrees(driveTrain.getHeading()));
        telemetry.addData("isBusy", driveTrain::isBusy);
    }

    @Override
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                turn(Math.toRadians(90)),
                new TankDriveCommand(driveTrain,
                        () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y)
        );
    }
}
