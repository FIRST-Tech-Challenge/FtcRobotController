package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FF_TrajectorySequence_Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final int NUM_CYCLES = 3;
        final int stepIncrement = 0;

        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-12, -59, Math.toRadians(90)); // x, y, heading (angle in radians)

        bot.setPoseEstimate(startPose);

        TrajectorySequence openingMove = bot.trajectorySequenceBuilder(startPose)
                .forward(15)
                .waitSeconds(1.5)
                .build();

        TrajectorySequence cycles = bot.trajectorySequenceBuilder(openingMove.end())
            .lineToLinearHeading(new Pose2d(10,-63.5, Math.toRadians(180)))
            .back(25 + stepIncrement)
            .waitSeconds(1)
            .forward(25 + stepIncrement)
            .lineToLinearHeading(new Pose2d(0,-42, Math.toRadians(125)))
            .waitSeconds(1.5)
            .build();

        TrajectorySequence hubToPark = bot.trajectorySequenceBuilder(cycles.end())
            .setReversed(true)
            .splineTo(new Vector2d(-59,-35), Math.toRadians(90))
            .build();

        waitForStart();
        bot.followTrajectorySequence(openingMove);
        for(int i = 0; i < NUM_CYCLES; i++) {
            if (!isStopRequested())
                bot.followTrajectorySequence(cycles);
        }
        bot.followTrajectorySequence(hubToPark);
    }
}
