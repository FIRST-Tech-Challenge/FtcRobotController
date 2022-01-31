package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class NewAutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoCarousel carousel = new AutoCarousel(hardwareMap, multiplier);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        final int[] height = {-1};

        final Pose2d initial = new Pose2d(-47, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);

        // Part 1: go to shipping hub
        final Pose2d p1Finish = new Pose2d(multiplier == 1 ? -21 : -20, multiplier == 1 ? 42 : -38, Math.toRadians(multiplier == 1 ? 100 : -95));
        final Trajectory part1;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(initial);
            builder.lineTo(new Vector2d(-40, multiplier == 1 ? 55 : -53));
            builder.splineToLinearHeading(p1Finish, Math.toRadians(-110 * multiplier));
            part1 = builder.build();
        }

        // Part 2: carousel
        final Pose2d p2Finish = new Pose2d(multiplier == 1 ? -60 : -60.5, multiplier == 1 ? 58 : -58.5, Math.toRadians(multiplier == 1 ? 240 : 330));
        final Trajectory part2;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(p1Finish);
            builder.lineTo(new Vector2d(-19, 50 * multiplier));
            builder.lineToLinearHeading(p2Finish);
            part2 = builder.build();
        }

        // Part 3: Park in Alliance Storage Unit
        final Pose2d p3Finish = new Pose2d(-60, 35 * multiplier, Math.toRadians(90 * multiplier));
        final Trajectory part3;
        {
            TrajectoryBuilder builder = drive.trajectoryBuilder(p2Finish);
            builder.lineToLinearHeading(p3Finish);
            part3 = builder.build();
        }

        waitForStart();

        // Part 1
        drive.followTrajectoryAsync(part1);
        while (!isStopRequested() && drive.isBusy()) {

        }
        if (isStopRequested()) {
            return;
        }

        // Part 2
        drive.followTrajectoryAsync(part2);
        while (!isStopRequested() && drive.isBusy()) {

        }
        if (isStopRequested()) {
            return;
        }

        // Part 3
        drive.followTrajectoryAsync(part3);
        while (!isStopRequested() && drive.isBusy()) {

        }
    }

    public void loop(Pose2d correctPosition) {

    }
}
