package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous
public class AutoWarehouse extends LinearOpMode {
    protected int multiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        TseDetector detector = new TseDetector(hardwareMap, "webcam", true);
        AtomicInteger height = new AtomicInteger(-1);
        // first marker shall cache the height
        final int[] cachedHeight = {-1};
        double nextToWall = 70 - inchesToCoordinate(5.8D);

        AutoIntake intake = new AutoIntake(hardwareMap);


        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        final Pose2d initial = new Pose2d(0, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 * multiplier));
        drive.setPoseEstimate(initial);

        TrajectorySequence startSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

            builder.lineToLinearHeading(new Pose2d(-3, 40 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addDisplacementMarker(() -> {
                // TODO ADD LIFT
                cachedHeight[0] = height.get();
            });
            builder.waitSeconds(2);
            builder.lineToLinearHeading(new Pose2d(-3, nextToWall * multiplier, Math.toRadians(0)));
            builder.lineTo(new Vector2d(20, nextToWall * multiplier));
            builder.lineTo(new Vector2d(40, nextToWall * multiplier));
            startSequence = builder.build();
        }

        TrajectorySequence secondSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d(40,
                    68 * multiplier, Math.toRadians(0)));
            builder.lineTo(new Vector2d(-3, nextToWall * multiplier));
            builder.lineToLinearHeading(new Pose2d(-3, 40 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addDisplacementMarker(() -> {
                // TODO LIFT UP
            });
            builder.waitSeconds(2);
            builder.addDisplacementMarker(() -> {
                // TODO LIFT DOWN
            });
            builder.lineToLinearHeading(new Pose2d(-3, 66 * multiplier, 0));
            builder.lineTo(new Vector2d(40, 70 * multiplier));

            secondSequence = builder.build();
        }

        Thread detectorThread = new Thread(() -> height.set(detector.run()));

        waitForStart();
        isStopRequested();

        detectorThread.start();

        if (!isStopRequested()) {

            drive.followTrajectorySequence(startSequence);

            for (int i = 0; i < 3; i++) {
//                // intake code
//                intake.forward();
//                while (!intake.containsObject()) {
//                    drive.setMotorPowers(1, 1, -1,-1);
//                }
//                intake.stop();
//                // You'll want to correct for the distance that made it travel
//                drive.setMotorPowers(0, 0, 0, 0);
//                drive.followTrajectory(
//                        drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(40, 68 * multiplier,
//                                        Math.toRadians(0)))
//                                .build()
//                );
                drive.followTrajectorySequence(secondSequence);
            }

        }
    }
}
