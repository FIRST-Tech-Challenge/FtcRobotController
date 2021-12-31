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
public class AutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public int directionAdder = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d(-40, multiplier * 70 - inchesToCoordinate(9),
                Math.toRadians(90 + directionAdder)));

        builder.waitSeconds(0.1);
        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, 55));
        builder.splineToLinearHeading(new Pose2d(-20, 40, Math.toRadians(-110)),
                Math.toRadians(-110));
        builder.addDisplacementMarker(() -> {
            // TODO LIFT UP
        });
        builder.waitSeconds(2);
        builder.addDisplacementMarker(() -> {
            // TODO LIFT DOWN
        });
        builder.waitSeconds(2);
        builder.lineTo(new Vector2d(-19, 45));
        builder.lineToLinearHeading(new Pose2d(-59, 57.5, Math.toRadians(240)));
        builder.addDisplacementMarker(() -> {
            // TODO START CAROSUEL
        });
        builder.waitSeconds(3);
        builder.addDisplacementMarker(() -> {
            // TODO STOP CAROSUEL
        });
        builder.lineToLinearHeading(new Pose2d(-60, 35, Math.toRadians(90)));
        builder.waitSeconds(10);
        builder.addDisplacementMarker(() -> {
            // TODO FIRE TAPE MEASURE
        });
        builder.waitSeconds(3);

        TrajectorySequence trajSeq = builder.build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
    }
}
