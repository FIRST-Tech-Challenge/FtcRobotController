package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.TimedEvent;
import org.firstinspires.ftc.teamcode.opmodes.util.PoseStorage;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
@Disabled
public class OldAutoWarehouse extends LinearOpMode {
    protected int multiplier = 1;
    protected boolean isRed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        TseDetector detector = new TseDetector(hardwareMap, "webcam", true, isRed);
        final int[] height = {-1};
        double nextToWall = 70 - inchesToCoordinate(5.8D);

        MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry);

        EventThread eventThread = new EventThread(() -> !isStopRequested());

        AutoIntake intake = new AutoIntake(hardwareMap, eventThread);
        AutoLift lift = new AutoLift(eventThread, hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        final Pose2d initial = new Pose2d(0, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 * multiplier));
        drive.setPoseEstimate(initial);

        TrajectorySequence startSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);
            builder.lineTo(new Vector2d(-3, 58 * multiplier));
            // Go to lift
            builder.lineToLinearHeading(new Pose2d(-2, 43.5 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addTemporalMarker(() -> {
                lift.setPosition(getPosition(height[0]));
            });
            builder.waitSeconds(3.75);
            builder.lineToLinearHeading(new Pose2d(0, (nextToWall + 1) * multiplier));
            builder.addTemporalMarker(() -> drive.setWeightedDrivePower(new Pose2d(0, -0.2 * multiplier, 0)));
            builder.lineTo(new Vector2d(20, (nextToWall + 1) * multiplier));
            // go to warehouse
            builder.lineTo(new Vector2d(40, (nextToWall + 1) * multiplier));
            builder.addTemporalMarker(() -> drive.setWeightedDrivePower(new Pose2d(0, 0, 0)));
            startSequence = builder.build();
        }

        TrajectorySequence secondSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d(40,
                    nextToWall * multiplier, Math.toRadians(0)));
            builder.lineTo(new Vector2d(-3, nextToWall * multiplier));
            builder.lineToLinearHeading(new Pose2d(-2, 43.5 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addTemporalMarker(() -> {
                lift.setPosition(AutoLift.Positions.TOP);
            });
            builder.waitSeconds(4);
            builder.lineToLinearHeading(new Pose2d(0, (nextToWall + 1) * multiplier));
            builder.addTemporalMarker(() -> drive.setWeightedDrivePower(new Pose2d(0, -0.2 * multiplier, 0)));
            builder.lineTo(new Vector2d(40, (nextToWall + 1) * multiplier));
            builder.addTemporalMarker(() -> drive.setWeightedDrivePower(new Pose2d(0, 0, 0)));

            secondSequence = builder.build();
        }

        Thread thread = new Thread(() -> {
            while (!isStopRequested()) {
                lift.update();
            }
        });

        waitForStart();
        height[0] = detector.run();
        goodTelemetry.addData("height", height[0]);
        goodTelemetry.update();

        thread.start();
        eventThread.start();

        if (!isStopRequested()) {

            drive.followTrajectorySequence(startSequence);

            for (int i = 0; i < 1; i++) {
                // intake a block
                intake.backward();
                drive.setWeightedDrivePower(new Pose2d(0.2, 0, 0));
                while (intake.noObject()) {
                    if (isStopRequested()) {
                        return;
                    }
                }
                eventThread.addEvent(new TimedEvent(intake::stop, 250));
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                drive.followTrajectory(
                        drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(40, nextToWall + 2 * multiplier,
                                        Math.toRadians(0)))
                                .build()
                );

                {
                    Pose2d pose = drive.getPoseEstimate();
                    drive.setPoseEstimate(new Pose2d(pose.getX(), nextToWall * multiplier, pose.getHeading()));
                }

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
