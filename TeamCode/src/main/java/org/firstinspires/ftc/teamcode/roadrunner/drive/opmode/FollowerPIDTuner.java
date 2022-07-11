package org.firstinspires.ftc.teamcode.roadrunner.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
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
 */
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
