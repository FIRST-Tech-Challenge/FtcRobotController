package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AutoWarehouse extends LinearOpMode {
    protected int multiplier = 1;
    protected boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {

        TseDetector detector = new TseDetector(hardwareMap, "webcam", true);
        final int[] height = {-1};
        double nextToWall = 70 - inchesToCoordinate(5.8D);

        EventThread eventThread = new EventThread();

        AutoIntake intake = new AutoIntake(hardwareMap);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        final Pose2d initial = new Pose2d(0, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 * multiplier));
        drive.setPoseEstimate(initial);

        TrajectorySequence startSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

            builder.lineToLinearHeading(new Pose2d(-1, 41.5 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addTemporalMarker(() -> lift.setPosition(getPosition(height[0])));
            builder.waitSeconds(4);
            builder.addTemporalMarker(() -> lift.setPosition(AutoLift.Positions.INTAKING));
            builder.lineToLinearHeading(new Pose2d(0, nextToWall * multiplier, Math.toRadians(3)));
            builder.lineTo(new Vector2d(20, nextToWall * multiplier));
            builder.lineTo(new Vector2d(40, nextToWall * multiplier));
            startSequence = builder.build();
        }

        TrajectorySequence secondSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d(40,
                    nextToWall * multiplier, Math.toRadians(0)));
            builder.lineTo(new Vector2d(-3, nextToWall * multiplier));
            builder.lineToLinearHeading(new Pose2d(-1, 41.5 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addTemporalMarker(() -> lift.setPosition(getPosition(height[0])));
            builder.waitSeconds(8);
            builder.lineToLinearHeading(new Pose2d(0, nextToWall * multiplier, Math.toRadians(3)));
            builder.lineTo(new Vector2d(40, nextToWall * multiplier));

            secondSequence = builder.build();
        }

        Thread thread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
            }
        });

        waitForStart();
        isStopRequested();

        thread.start();

        height[0] = detector.run(isRed);

        if (!isStopRequested()) {

            drive.followTrajectorySequence(startSequence);

            for (int i = 0; i < 3; i++) {
                // intake code
                intake.backward();
                drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);
                while (intake.noObject()) {
                    if (isStopRequested()) {
                        return;
                    }
                }
                intake.stop();
                // You'll want to correct for the distance that made it travel
                drive.setMotorPowers(0, 0, 0, 0);
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(40, nextToWall + 2 * multiplier,
                                        Math.toRadians(0)))
                                .build()
                );

                drive.followTrajectorySequence(secondSequence);
            }
        }
        drive.followTrajectorySequenceAsync(null);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    public AutoLift.Positions getPosition(int input) {
        return input == 1 ? AutoLift.Positions.BOTTOM :
                input == 2 ? AutoLift.Positions.MIDDLE : AutoLift.Positions.TOP;
    }
}
