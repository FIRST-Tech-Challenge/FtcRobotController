package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.GFORCE_KiwiDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        GFORCE_KiwiDrive drive = new GFORCE_KiwiDrive(hardwareMap);

        Trajectory trajectoryForward = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryLeft = drive.trajectoryBuilder(trajectoryForward.end())
                .strafeLeft(DISTANCE/2)
                .build();

        Trajectory trajectoryBackward = drive.trajectoryBuilder(trajectoryLeft.end())
                .back(DISTANCE)
                .build();

        Trajectory trajectoryRight = drive.trajectoryBuilder(trajectoryBackward.end())
                .strafeRight(DISTANCE/2)
                .build();

        TrajectorySequence trajectorySquare = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(DISTANCE)
                .strafeLeft(DISTANCE/2)
                .back(DISTANCE)
                .strafeRight(DISTANCE/2)
                .build();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drive.followTrajectory(trajectoryForward);
            drive.followTrajectory(trajectoryLeft);
            drive.followTrajectory(trajectoryBackward);
            drive.followTrajectory(trajectoryRight);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading ODO. ", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("heading GYRO.", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();

            while (opModeIsActive() && !gamepad1.y) {};

            drive.followTrajectorySequence(trajectorySquare);

            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading ODO. ", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("heading GYRO.", Math.toDegrees(drive.getExternalHeading()));
            telemetry.update();

        }
    }



}