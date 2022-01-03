package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoIntake;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AutoWarehouse extends LinearOpMode {
    protected int multiplier = 1;

    @Override
    public void runOpMode() throws InterruptedException {
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
            builder.waitSeconds(2);
            builder.lineToLinearHeading(new Pose2d(-3, 60 * multiplier, Math.toRadians(0)));
            builder.lineTo(new Vector2d(20, 64 * multiplier));
            builder.lineTo(new Vector2d(40, 64 * multiplier));

            startSequence = builder.build();
        }

        TrajectorySequence secondSequence;

        {
            TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d(40,
                    64 * multiplier, Math.toRadians(0)));
            builder.lineTo(new Vector2d(-3, 64 * multiplier));
            builder.lineToLinearHeading(new Pose2d(-3, 40 * multiplier,
                    Math.toRadians(70 * multiplier)));
            builder.addDisplacementMarker(() -> {
                // TODO LIFT UP
            });
            builder.waitSeconds(2);
            builder.addDisplacementMarker(() -> {
                // TODO LIFT DOWN
            });
            builder.lineToLinearHeading(new Pose2d(-3, 64 * multiplier, 0));
            builder.lineTo(new Vector2d(40, 64 * multiplier));

            secondSequence = builder.build();
        }

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(startSequence);

            for (int i = 0; i < 3; i++) {
                // intake code
//                intake.forward();
//                while (!intake.containsObject()) {
//                    drive.setMotorPowers(1, 1, -1,-1);
//                }
//                intake.stop();
//                // You'll want to correct for the distance that made it travel
//                drive.followTrajectory(
//                        drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToLinearHeading(new Pose2d(40, 64 * multiplier,
//                                        Math.toRadians(0)))
//                                .build()
//                );
                drive.followTrajectorySequence(secondSequence);
            }

        }
    }
}
