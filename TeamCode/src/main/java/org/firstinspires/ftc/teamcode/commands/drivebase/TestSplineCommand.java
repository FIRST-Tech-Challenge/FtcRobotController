package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.technototes.library.command.Command;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

/*
 * This is an example of a more complex path to really test the tuning.
 */
public class TestSplineCommand extends Command {
    public DrivebaseSubsystem subsystem;
    public Trajectory traj, traj2;

    public TestSplineCommand(DrivebaseSubsystem sub) {
        addRequirements(sub.dummySubsystem);
        subsystem = sub;
        traj = subsystem.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), 0)
                .build();
        traj2 = subsystem.trajectoryBuilder(traj.end(), true)
                .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                .build();
    }

    @Override
    public void init() {
        subsystem.followTrajectoryAsync(traj);
    }

    @Override
    public void execute() {
        subsystem.update();
    }

    @Override
    public boolean isFinished() {
        return subsystem.getPoseEstimate().epsilonEquals(traj.end());
    }

    @Override
    public void end(boolean cancel) {
        subsystem.setDriveSignal(new DriveSignal());
    }
}
//    @Override
////    public void runOpMode() throws InterruptedException {
////        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
////
////        waitForStart();
////
////        if (isStopRequested()) return;
////
////        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
////                .splineTo(new Vector2d(70, 30), 0)
////                .build();
////
////
////        drive.followTrajectory(traj);
////        sleep(2000);
////
////        drive.followTrajectory(
////                drive.trajectoryBuilder(traj.end(), true)
////                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
////                        .build()
////        );
////    }
