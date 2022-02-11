package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.util.VisionToLiftHeight.getPosition;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.opmodes.util.StayInPosition.stayInPose;

@Autonomous
public class NewAutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry goodTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoCarousel carousel = new AutoCarousel(hardwareMap, multiplier);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        int height;

        final Pose2d initial = new Pose2d(-47, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);

        final boolean[] liftUpdated = {false};
        Thread liftThread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
                liftUpdated[0] = true;
            }
        });

        ElapsedTime toolTimer = new ElapsedTime();

        // Part 1: go to shipping hub
        final TrajectorySequence part1 = drive.trajectorySequenceBuilder(initial)
            .lineTo(new Vector2d(-20, 55 * multiplier))
            .lineToLinearHeading(new Pose2d(-21,
                    42 * multiplier, Math.toRadians(95 * multiplier)))
            .build();

        // Part 2: carousel
        final Trajectory part2 = drive.trajectoryBuilder(part1.end())
            .splineToSplineHeading(new Pose2d(-60.5, 60 * multiplier,
                    Math.toRadians(!isRed ? 240 : 330)), Math.toRadians(180))
            .build();


        // Part 3: Park in Alliance Storage Unit
        final Trajectory part3 = drive.trajectoryBuilder(part2.end())
                .lineToLinearHeading(new Pose2d(-60, 35 * multiplier,
                        Math.toRadians(90 * multiplier)))
                .build();

        waitForStart();
        liftThread.start();
        eventThread.start();

        height = 3; // detector.run();
        goodTelemetry.addData("height", height);
        goodTelemetry.update();

        // Part 1
        drive.followTrajectorySequenceAsync(part1);
        updateLoop(drive);
        if (isStopRequested()) return;

        liftUpdated[0] = false;
        lift.setPosition(getPosition(height));
        toolTimer.reset();
        while (toolTimer.seconds() < 3) {
            if (isStopRequested()) {
                return;
            }
            stayInPose(drive, part1.end());
        }

        // Part 2
        drive.followTrajectoryAsync(part2);
        updateLoop(drive);
        if (isStopRequested()) return;
        Pose2d part2endPose = drive.getPoseEstimate();

        // CAROUSEL GARBAG
        carousel.on();
        toolTimer.reset();
        while (!isStopRequested() && toolTimer.seconds() < 4) {
            stayInPose(drive, part2endPose);
        }
        if (isStopRequested()) return;
        carousel.off();

        // Part 3
        drive.followTrajectoryAsync(part3);
        updateLoop(drive);
        while (!isStopRequested()) {
            stayInPose(drive, part3.end());
            drive.update();
        }
    }

    public void updateLoop(SampleMecanumDrive drive) {
        while (!isStopRequested() && drive.isBusy()) {
            drive.update();
        }
    }
}
