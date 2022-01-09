package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.RunCommand;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TurnCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;

/**
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * NOTE: this has been refactored to use FTCLib's command-based
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends CommandOpMode {

    public static double DISTANCE = 48; // in

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand followerCommand;
    private Command trajGroup;
    private TurnCommand turnCommand;
    private Pose2d startPose;

    @Override
    public void initialize() {
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        schedule(new RunCommand(() -> {
            if (trajGroup == null || trajGroup.isFinished() || !trajGroup.isScheduled()) {
                Trajectory traj = drive.trajectoryBuilder(startPose)
                        .forward(DISTANCE)
                        .build();
                followerCommand = new TrajectoryFollowerCommand(drive, traj);
                turnCommand = new TurnCommand(drive, Math.toRadians(90));
                trajGroup = new SequentialCommandGroup(followerCommand, turnCommand)
                        .whenFinished(() -> startPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(90))));
                trajGroup.schedule();
            }
        }));
    }

}
