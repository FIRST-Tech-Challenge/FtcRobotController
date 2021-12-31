package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AutoWarehouse extends LinearOpMode {
    public int multiplier = 1;
    public int directionAdder = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d(0, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 + directionAdder)));

        builder.lineToLinearHeading(new Pose2d(-3, 40, Math.toRadians(70)));
        builder.waitSeconds(2);
        builder.lineToLinearHeading(new Pose2d(-3, 60, Math.toRadians(0)));
        builder.lineTo(new Vector2d(20, 64));
        builder.lineTo(new Vector2d(40, 64));
        for (int i = 0; i < 4; i++) {
            builder.lineTo(new Vector2d(-3,64));
            builder.lineToLinearHeading(new Pose2d(-3, 40, Math.toRadians(70)));
            builder.addDisplacementMarker(() -> {
                // TODO LIFT UP
            });
            builder.waitSeconds(2);
            builder.addDisplacementMarker(() -> {
                // TODO LIFT DOWN
            });
            builder.lineToLinearHeading(new Pose2d(-3,64,0));
            builder.lineTo(new Vector2d(40,64));
        }

        TrajectorySequence trajSeq = builder.build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
