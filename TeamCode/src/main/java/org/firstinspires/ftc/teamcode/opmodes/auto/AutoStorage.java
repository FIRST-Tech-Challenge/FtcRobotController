package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.RoadRunnerHelper.inchesToCoordinate;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class AutoStorage extends LinearOpMode {
    public int multiplier = 1;
    public int directionAdder = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        AutoCarousel carousel = new AutoCarousel(hardwareMap);

        final Pose2d initial = new Pose2d(-40, multiplier * (70 - inchesToCoordinate(9)),
                Math.toRadians(90 * multiplier));

        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);
        drive.setPoseEstimate(initial);
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(initial);

        builder.waitSeconds(0.1);
        // 9.35 seconds long
        builder.lineTo(new Vector2d(-40, 55 * multiplier));
        builder.splineToLinearHeading(new Pose2d(-20, 40, Math.toRadians(-110 * multiplier)),
                Math.toRadians(-110));
        builder.addDisplacementMarker(() -> {
            // TODO LIFT UP
        });
        builder.waitSeconds(2);
        builder.addDisplacementMarker(() -> {
            // TODO LIFT DOWN
        });
        builder.waitSeconds(2);
        builder.lineTo(new Vector2d(-19, 45 * multiplier));
        builder.lineToLinearHeading(new Pose2d(-59, 57.5, Math.toRadians(240)));
        builder.addDisplacementMarker(carousel::on);
        builder.waitSeconds(3);
        builder.addDisplacementMarker(carousel::off);
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
